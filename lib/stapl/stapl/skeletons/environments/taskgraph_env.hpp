/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_ENVIRONMENTS_TASKGRAPH_HPP
#define STAPL_SKELETONS_ENVIRONMENTS_TASKGRAPH_HPP

#include <stapl/views/metadata/localize_object.hpp>
#include <stapl/views/localize_element.hpp>
#include <stapl/skeletons/utility/factory_add_task_helper.hpp>
#include <stapl/skeletons/flows/producer_info.hpp>
#include <stapl/skeletons/executors/paragraph_skeleton_manager_fwd.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/runtime/executor/scheduler/sched.hpp>
#include <stapl/paragraph/paragraph_view.hpp>

namespace stapl {
namespace skeletons {

//////////////////////////////////////////////////////////////////////
/// @brief The main environment for spawning skeletons in STAPL and
/// evaluating them as taskgraphs is @c taskgraph_env. It
/// gets the workfunction, unwraps the inputs which are given as
/// @c producer_info (@c indexed_producer, @c view_element_producer,
/// etc.) and calls @c add_task or @c set_num_succs from the given
/// @c PARAGRAPH via the given taskgraph view (@c TGV)
///
/// @tparam TGV taskgraph view that can be used to create tasks and
///             modify their metadata in STAPL
///
/// @ingroup skeletonsEnvironments
///////////////////////////////////////////////////////////////////////
template <typename TGV>
class taskgraph_env
{
  TGV                  m_tgv;
  std::size_t          m_num_PEs;
  runtime::location_id m_PE_id;

public:
  explicit taskgraph_env(TGV tgv)
    : m_tgv(tgv),
      m_num_PEs(-1),
      m_PE_id(-1)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Spawns an element in a given taskgraph. It first unwraps
  /// the input given as various types of @c producer_info and converts
  /// them to acceptable arguments to @c PARAGRAPH's input arguments for
  /// @c add_task requests.
  ///
  /// @param tid       the unique task id assigned for this element
  /// @param result_id result_id of this task if it has one
  /// @param wf        the workfunction to be executed by the task
  /// @param mapper    the output to output mapper for mapping the results
  ///                  of the child paragraph to current paragraph
  /// @param num_succs the exact number of successors for this task.
  ///                  Remember that in some cases this value is set to
  ///                  @c stapl::defer_specs (when the @c spawner is in
  ///                  @c SET_HOLD mode)
  /// @param in        inputs
  ///
  /// @note The enable_if in here is required only by gcc 4.8.2 and older
  /// versions.
  //////////////////////////////////////////////////////////////////////
  template <bool isResult, typename WF, typename Mapper, typename... In>
  typename std::enable_if<
    !skeletons::is_nested_skeleton<typename std::decay<WF>::type>::value,
    void>::type
  spawn_element(std::size_t tid,
                std::size_t result_id,
                WF&& wf,
                Mapper&& mapper,
                std::size_t num_succs,
                In&&... in)
  {
      add_task_helper<typename std::decay<WF>::type>()(
      m_tgv, tid, std::forward<WF>(wf), num_succs,
      make_producer(std::forward<In>(in))...);

    set_result_task_id(
      std::integral_constant<bool, isResult>(),
      is_skeleton<typename std::decay<WF>::type>(),
      mapper,
      result_id,
      tid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief A specialization of spawn_element for the case that the
  /// given workfunction is a skeleton itself and the skeleton is not
  /// reducing to a single element. Examples of such are @c zip(zip(op)),
  /// @c map(butterfly(op)).
  ///
  /// Internally, nested skeletons are wrapped inside the adapter class
  /// called @c wrapped_skeleton, which contains the preference for
  /// execution.
  ///
  /// @param tid       the unique task id assigned for this element
  /// @param result_id result_id of this task if it has one
  /// @param wf        a wrapped_skeleton with a @ref nested_execution
  ///                  execution tag
  /// @param mapper    the output to output mapper for mapping the results
  ///                  of the child paragraph to current paragraph
  /// @param num_succs the exact number of successors for this task.
  ///                  Remember that in some cases this value is set to
  ///                  @c stapl::defer_specs (when the @c spawner is in
  ///                  @c SET_HOLD mode)
  /// @param in        inputs
  //////////////////////////////////////////////////////////////////////
  template <bool isResult, typename S, bool Reducer, typename ExecutionParams,
            bool B, typename Mapper, typename... In>
  void spawn_element(
         std::size_t tid,
         std::size_t result_id,
         wrapped_skeleton<S, tags::nested_execution<Reducer>, ExecutionParams,
                          B> const& wf,
         Mapper&& mapper,
         std::size_t num_succs,
         In&&... in)
  {
    using wf_t =
     wrapped_skeleton<S, tags::nested_execution<Reducer>, ExecutionParams, B>;
    using optimizer_t = typename wf_t::optimizer_t;
    using return_t = typename wf_t::template result<optimizer_t(
      typename In::value_type...)>::type;

    auto&& exec_params = wf.get_exec_params();
    auto&& f =
      make_paragraph_skeleton_manager(
        wf.get_skeleton(),
        execution_params<return_t>(exec_params.coarsener(),
                                   exec_params.extra_env(),
                                   exec_params.scheduler())
      );

    spawn_element<isResult>(tid, result_id, f, std::forward<Mapper>(mapper),
                            num_succs, std::forward<In>(in)...);
  }
  //////////////////////////////////////////////////////////////////////
  /// @brief A specialization of spawn_element where the notification
  /// dependencies are also specified (@c notifications).
  ///
  /// @param tid           the unique task id assigned for this element
  /// @param result_id     result_id of this task if it has one
  /// @param notifications the list of task ids which this task depends on
  /// @param wf            the workfunction to be executed by the task
  /// @param mapper        the output to output mapper for mapping the results
  ///                      of the child paragraph to current paragraph
  /// @param num_succs     the exact number of successors for this task.
  ///                      Remember that in some cases this value is set to
  ///                      @c stapl::defer_specs (when the @c spawner is in
  ///                      @c SET_HOLD mode)
  /// @param in            inputs
  //////////////////////////////////////////////////////////////////////
  template <bool isResult, typename WF,
            typename Mapper, typename ...In>
  void spawn_element(std::size_t tid,
                     std::size_t result_id,
                     std::vector<std::size_t> const& notifications,
                     WF&& wf,
                     Mapper&& mapper,
                     std::size_t num_succs,
                     In&&... in)
  {
    add_task_helper<typename std::decay<WF>::type>()(
      m_tgv, tid, std::forward<WF>(wf), notifications, num_succs,
      make_producer(std::forward<In>(in))...);

    set_result_task_id(std::integral_constant<bool, isResult>(),
                       is_skeleton<typename std::decay<WF>::type>(),
                       mapper,
                       result_id,
                       tid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Spawn a task generated by a farm.
  ///
  /// @param wf the workfunction to be executed by the task
  /// @param in inputs
  //////////////////////////////////////////////////////////////////////
  template <typename WF, typename Farm, typename ...In>
  void spawn_farm_element(WF&& wf, Farm&& farm, In&&... in)
  {
    m_tgv.add_task(
      std::forward<WF>(wf),
      make_producer(std::forward<Farm>(farm)),
      make_producer(std::forward<In>(in))...);
  }

  template <typename... Args>
  void pre_spawn(Args&&...) const
  { }

  template <typename... Args>
  void post_spawn(Args&&...) const
  { }

  void set_num_succs(std::size_t tid, std::size_t num_succs) const
  {
    m_tgv.set_num_succs(tid, num_succs);
  }

  void init_location_info(std::size_t num_PEs,
                          runtime::location_id loc_id)
  {
    m_num_PEs = num_PEs;
    m_PE_id = loc_id;
  }

  std::size_t get_num_PEs() const
  {
    return m_num_PEs;
  }

  runtime::location_id get_PE_id() const
  {
    return m_PE_id;
  }

  void define_type(typer& t)
  {
    t.member(m_tgv);
    t.member(m_num_PEs);
    t.member(m_PE_id);
  }
private:

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the result id for this task if it has one.
  /// A specialization when the task is not a nested skeleton
  /// and current skeleton should set its results
  ///
  ///
  /// @param mapper    the mapper that is used for mapping the
  ///                  result ids of the task skeleton to the
  ///                  result ids of current skeleton.
  /// @param result_id result_id of this task if it has one
  /// @param tid       the unique task id assigned for this element
  //////////////////////////////////////////////////////////////////////
  template <typename Mapper>
  void set_result_task_id(std::true_type, std::false_type, Mapper&& mapper,
                          std::size_t result_id, std::size_t tid)
  {
    if (result_id != std::numeric_limits<std::size_t>::max())
    {
      m_tgv.set_result((std::size_t)result_id, tid);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the result id for this task if it has one.
  /// A specialization when the task is a nested skeleton
  /// and current skeleton should set its results
  ///
  ///
  /// @param mapper    the mapper that is used for mapping the
  ///                  result ids of the task skeleton to the
  ///                  result ids of current skeleton.
  /// @param result_id result_id of this task if it has one
  /// @param tid       the unique task id assigned for this element
  //////////////////////////////////////////////////////////////////////
  template <typename Mapper>
  void set_result_task_id(std::true_type, std::true_type, Mapper&& mapper,
                          std::size_t result_id, std::size_t tid)
  {
    if (result_id != std::numeric_limits<std::size_t>::max())
      m_tgv.set_result_pg(tid, mapper);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the result id for this task if it has one.
  /// A specialization when the current skeleton shouldn't set it's results.
  ///
  ///
  /// @param mapper    the mapper that is used for mapping the
  ///                  result ids of the task skeleton to the
  ///                  result ids of current skeleton.
  /// @param result_id result_id of this task if it has one
  /// @param tid       the unique task id assigned for this element
  //////////////////////////////////////////////////////////////////////
  template <typename Mapper>
  void set_result_task_id(std::false_type, bool /*is_skeleton*/,
                          Mapper&& mapper, std::size_t result_id,
                          std::size_t tid)
  { }

  template <typename V, typename I>
  typename stapl::result_of::consume<
             typename flows::indexed_producer<
               V, I, skeletons::no_filter, false
             >::value_type
           >::type
  make_producer(flows::indexed_producer<
                  V, I, skeletons::no_filter, false> const& producer) const
  {
    return consume<V>(m_tgv, producer.get_index());
  }

  template <typename V, typename I>
  auto
  make_producer(flows::indexed_producer<
                  V, I, skeletons::no_filter, true> const& producer) const
  ->decltype(consume_pg<V> (m_tgv, producer.get_index()))
  {
    return consume_pg<V>(m_tgv, producer.get_index());
  }

  template <typename V, typename I, typename Filter>
  auto
  make_producer_helper(std::false_type,
                flows::indexed_producer<
                  V, I, Filter, true
                > const& producer) const
  ->decltype(consume_pg<V>(std::declval<TGV>(), producer.get_index(),
             producer.get_filter().get_shouldflow_filter(),
             producer.get_filter().get_value_filter()))
  {
    return consume_pg<V>(
             m_tgv, producer.get_index(),
             producer.get_filter().get_shouldflow_filter(),
             producer.get_filter().get_value_filter()
             );
  }

  template <typename V, typename I, typename Filter>
  auto
  make_producer_helper(std::true_type,
                flows::indexed_producer<
                  V, I, Filter, true
                > const& producer) const
  ->decltype(consume_pg<V>(std::declval<TGV>(), producer.get_index(),
             producer.get_filter().get_shouldflow_filter()))
  {
    return consume_pg<V>(
             m_tgv, producer.get_index(),
             producer.get_filter().get_shouldflow_filter()
             );
  }

  template <typename V, typename I, typename Filter>
  auto
  make_producer(flows::indexed_producer<
                  V, I, Filter, true
                > const& producer) const
  ->decltype(this->make_producer_helper(
                     std::declval<typename std::is_same<
                      typename std::decay<
                        typename Filter::value_filter_type>::type,
                      skeletons::no_filter>::type>(),
                     producer))
  {
    return make_producer_helper(
      typename std::is_same<
        typename std::decay<typename Filter::value_filter_type>::type,
        skeletons::no_filter>::type(),
      producer);
  }

  template <typename V, typename I, typename F>
  typename stapl::result_of::consume<
             typename flows::indexed_producer<V, I, F>::value_type,
             typename flows::indexed_producer<V, I, F>::filter_type>::type
  make_producer(flows::indexed_producer<V, I, F> const& producer) const
  {
    return consume<V>(m_tgv,
                      producer.get_index(),
                      producer.get_filter());
  }

  template <typename V, typename I, typename F>
  typename stapl::result_of::aggregated_consume<
             typename flows::indexed_producer<V, I, F>::value_type,
             typename flows::indexed_producer<V, I, F>::filter_type>::type
  make_producer(flows::indexed_producers<V, I, F> const& producer) const
  {
    return consume<V>(m_tgv,
                      producer.get_indices(),
                      producer.get_filter());
  }

  template <typename T, typename Mapper,
            typename Index, typename... OptionalValueFilter>
  auto
  make_producer(flows::paragraph_producer<
                  nested_pg_view_subview<T, OptionalValueFilter...>,
                    Index, false, Mapper> const& producer) const
    -> decltype(consume<T>(producer.get_element(), std::declval<size_t>()))
  {
    auto&& mapped_idx =
      producer.get_mapper().get_out_to_in_mapper()(producer.get_index());

    producer.get_element().set_num_succs(
      mapped_idx,
      producer.consumer_count(producer.get_index()));

    return consume<T>(producer.get_element(), mapped_idx);
  }

  template <typename T, typename Index,
            typename Mapper, typename... OptionalValueFilter>
  std::pair<ptr_wrapper<pg_edge_view<T>>, std::size_t>
  make_producer(flows::paragraph_producer<
                  nested_parent_pg_view_subview<T, OptionalValueFilter...>,
                    Index, false, Mapper> const& producer) const
  {
    auto&& mapped_idx =
      producer.get_mapper().get_out_to_in_mapper()(producer.get_index());

    producer.get_element().set_num_succs(
      mapped_idx,
      producer.consumer_count(producer.get_index()));

    return consume<T>(producer.get_element(), mapped_idx);
  }

  template <typename T, typename Mapper,
            typename Index, typename... OptionalValueFilter>
  auto
  make_producer(flows::paragraph_producer<
                  nested_pg_view_subview<T, OptionalValueFilter...>,
                    Index, true, Mapper> const& producer) const
  -> decltype(consume_pg<T>(producer.get_element(),
                            std::vector<std::size_t>(),
                            producer.get_mapper().get_in_to_in_mapper()))
  {
    auto&& in_to_in_mapper = producer.get_mapper().get_in_to_in_mapper();
    std::vector<std::size_t> pin_ids;

    for (auto&& i : in_to_in_mapper.get_result_ids())
    {
      pin_ids.push_back(i);
      producer.get_element().set_num_succs(i, 1);
    }

    return consume_pg<T>(producer.get_element(), pin_ids, in_to_in_mapper);
  }

  template <typename T, typename Mapper,
            typename Index, typename... OptionalValueFilter>
  auto
  make_producer(flows::paragraph_producer<
                  nested_parent_pg_view_subview<T, OptionalValueFilter...>,
                    Index, true, Mapper> const& producer) const
  -> decltype(consume_pg<T>(producer.get_element(),
                            std::vector<std::size_t>(),
                            producer.get_mapper().get_in_to_in_mapper()))
  {
    auto&& in_to_in_mapper = producer.get_mapper().get_in_to_in_mapper();
    std::vector<std::size_t> pin_ids;

    for (auto&& i : in_to_in_mapper.get_result_ids())
    {
      /* todo: use consumer count, right now just hack */
      pin_ids.push_back(i);
      producer.get_element().set_num_succs(i, 1);
    }

    return consume_pg<T>(producer.get_element(), pin_ids, in_to_in_mapper);
  }

  template <typename T, typename A>
  localize_element<
    typename flows::view_element_producer<
      proxy<T, A>>::value_type::value_type,
    std::size_t,
    stub_dontcare<
      typename flows::view_element_producer<
        proxy<T, A>>::value_type::value_type,
      std::size_t> >
  make_producer(
    flows::view_element_producer<proxy<T, A>> const& producer) const
  {
    return localize_ref(
             producer.get_element().operator[](producer.get_index()));
  }

  template <typename View>
  std::pair<View*, typename flows::view_element_producer<View>::index_type>
  make_producer(flows::view_element_producer<View&> const& producer) const
  {
    typedef std::pair<
      View*,
      typename flows::view_element_producer<View&>::index_type> producer_t;
    return producer_t(&producer.get_element(), producer.get_index());
  }

  template <typename View>
  auto
  make_producer(flows::view_element_producer<View*> const& producer) const
    -> decltype(localize_ref(producer.get_element(), producer.get_index()))
  {
    return localize_ref(producer.get_element(), producer.get_index());
  }


  template <typename Element>
  auto
  make_producer(flows::reflexive_producer<Element&> const& producer) const
    -> decltype(localize_self_element(producer.get_element()))
  {
    return localize_self_element(producer.get_element());
  }

  template <typename Element>
  auto
  make_producer(flows::reflexive_producer<Element> const& producer) const
    -> decltype(localize_ref(producer.get_element()))
  {
    return localize_ref(producer.get_element());
  }

  template <typename Element>
  typename stapl::result_of::localize_object<Element, true>::type
  make_producer(flows::constant_producer<Element> const& producer) const
  {
    return localize_object<true>(producer.get_element());
  }
};

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_ENVIRONMENTS_TASKGRAPH_HPP

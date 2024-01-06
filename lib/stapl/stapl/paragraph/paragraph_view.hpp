/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_PARAGRAPH_VIEW_HPP
#define STAPL_PARAGRAPH_PARAGRAPH_VIEW_HPP

#include <type_traits>

#include <stapl/utility/down_cast.hpp>
#include <stapl/runtime/type_traits/is_p_object.hpp>
#include <stapl/paragraph/paragraph_impl.hpp>
#include <stapl/paragraph/edge_container/views/edge_view.hpp>
#include <stapl/paragraph/edge_container/views/aggregated_edge_view.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Wraps a view object with a interface that makes it behave as a
///   pointer to a view of the same type would.
/// @ingroup pgView
///
/// This is used for PARAGRAPH edge views passed to @p add_task calls via
/// @p consume calls, where there is not a view copy held/owned by the PARAGRAPH
/// (i.e., those passed by user at PARAGRAPH creation).
///
/// When @ref paragraph_view::add_task pair usage is finally retired, there will
/// be no more need for this wrapper class.
///
/// @todo Default constructor should be removed.  For now, needed for
/// delazify transform for nested, out of gang migration.
///
/// @todo Aggregated subview use case may have unnecessary copy.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct ptr_wrapper
{
  View         m_vw;

  ptr_wrapper(void) = default;

  ptr_wrapper(View vw)
    : m_vw(std::move(vw))
  { }

  void define_type(typer& t)
  {
    t.member(m_vw);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Mimics the application operator* on a pointer / iterator.
  //////////////////////////////////////////////////////////////////////
  View& operator*()
  {
    return m_vw;
  }

  View const& operator*() const
  {
    return m_vw;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Mimics the application operator-> on a pointer / iterator.
  //////////////////////////////////////////////////////////////////////
  View const* operator->() const
  {
    return &m_vw;
  }
};


namespace paragraph_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Trivial view class for PARAGRAPH holding a @ref task_graph pointer.
///   Used for consume() statements and as a base class of @p Scheduler typed
///   class template @ref paragraph_view.
/// @ingroup pgView
//////////////////////////////////////////////////////////////////////
class paragraph_view_base
{
private:
  /// @brief Pointer to type erased base class of underlying PARAGRAPH.
  task_graph* m_paragraph_ptr;

public:
  task_graph& container(void) const
  {
    return *m_paragraph_ptr;
  }

  paragraph_view_base(task_graph& paragraph)
    : m_paragraph_ptr(&paragraph)
  { }

  void define_type(typer& t)
  {
    t.member(m_paragraph_ptr);
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @note this method is implemented as a workaround of Intel 14's
  /// default destructor when the code is compiled with -O0.  The default
  /// destructor provided incorrectly zeroes out memory, resulting in
  /// seg faults in task execution of dynamic work functions.
  //////////////////////////////////////////////////////////////////////
  ~paragraph_view_base() = default;
}; // class paragraph_view_base


//////////////////////////////////////////////////////////////////////
/// @brief View specification transformation functor used by the
///  @ref paragraph_view to include a rmi_handle::reference as well when
///  accepting and forwarding view specifications via @p add_task().
//////////////////////////////////////////////////////////////////////
template<typename V, typename = void>
struct transform_lazy_view
{
  using type = V;

  template<typename VParam>
  static VParam&& apply(VParam&& v)
  {
    return std::forward<VParam>(v);
  }
};


template<typename T1, typename T2>
struct transform_lazy_view<std::pair<T1*, T2>,
           typename std::enable_if<is_p_object<T1>::value>::type>
{
  using type = std::pair<p_object_pointer_wrapper<T1>, T2>;

  static type apply(std::pair<T1*, T2> const& v)
  {
    return type(v.first, v.second);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Provides an interface for a PARAGRAPH's factory as well as dynamic
///   tasks to add new tasks to the PARAGRAPH during execution.
/// @ingroup pgView
///
/// @tparam Scheduler The type of the scheduler that the PARAGRAPH was
/// constructed with.
///
/// Although this class only directly uses the scheduling metadata type
/// (i.e., Scheduler::sched_info_type) in its method signatures, it is
/// templated with the full Scheduler type so that the PARAGRAPH intermediate
/// base with knowledge of this type can be accessed.  This allows the
/// PARAGRAPH's internal @p add_task call to invoke the task placement policy
/// located in the scheduler object.
///
/// @todo Make scheduler template parameter optional, defaulting to
/// fifo_scheduler<default_task_placement>, to reduce symbol size.
//////////////////////////////////////////////////////////////////////
template<typename Scheduler>
class paragraph_view
  : public paragraph_view_base
{
private:
  using pred_list_t     = std::vector<std::size_t>;
  using sched_info_type = typename Scheduler::sched_info_type;
  using derived_tg_t    = task_graph_impl<Scheduler>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Extract a reference to the currently running PARAGRAPH from
  /// a dynamic call stack of PARAGRAPH invocations maintained in the runtime.
  ///
  /// @return A reference to an PARAGRAPH object whose type has factory and
  /// view information erased, but still maintains knowledge of the scheduler
  /// type.
  //////////////////////////////////////////////////////////////////////
public:
  derived_tg_t& graph() const
  {
    return down_cast<derived_tg_t>(this->container());
  }

public:
  paragraph_view(task_graph& paragraph)
    : paragraph_view_base(paragraph)
  { }

  void define_type(typer& t)
  {
    t.base<paragraph_view_base>(*this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Allows locations participating in a nested @ref paragraph that
  /// did not initiate the spawning process to convey knowledge about the
  /// number of nested paragraphs that they will participate in.  Used to
  /// aid in elision of global termination detection.
  //////////////////////////////////////////////////////////////////////
  void add_expected_tasks(size_t count) const
  {
    this->graph().count_task(count);
    this->graph().count_local_task(count);
    this->graph().pulse_terminator();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a new task in the currently running PARAGRAPH.
  ///
  /// @param task_id   User specified identifier that other tasks and
  ///                  @p paragraph_view methods use to refer to this task.
  /// @param wf        The workfunction for this task.
  /// @param num_succs The number of consumers for this task in the PARAGRAPH.
  /// @param v         Variadic list of view inputs to workfunction.
  ///
  /// @todo The number of method signatures for @p add_task is probably bigger
  /// than it needs to be.  See if we can simplify and synthesize.
  ///
  /// @note For this and all add_task signatures, the view parameter is actually
  /// a std::pair of View* and index which will be evaluated as conceptually as
  /// *view[index] prior to being passed to the workfunction.  This allows the
  /// actual evaluation of to be delayed until the task is actual placed,
  /// hopefully with good locality, so that view creation avoids communication.
  ///
  /// @todo Everyone agrees the std::pair interface should be made simpler,
  /// but we need find a balance between clean syntax and good performance.
  /// View, container, and PARAGRAPH component owners need to work together on
  /// this one.
  //////////////////////////////////////////////////////////////////////
  template<typename WF, typename... Args>
  void add_task(std::size_t task_id, WF const& wf, std::size_t num_succs,
                Args const&... args) const
  {
    this->graph().count_task();

    this->graph().add_task(
      task_id, wf, none, no_preds_t(), num_succs, LQ_LOOKUP,
      transform_lazy_view<typename std::decay<Args>::type>::
        apply(args)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a new task in the currently running PARAGRAPH.
  ///
  /// @param sched_info Scheduler metadata (e.g., priority) for task.
  /// @param task_id    User specified identifier that other tasks and
  ///                   @p paragraph_view methods use to refer to this task.
  /// @param wf         The workfunction for this task.
  /// @param num_succs  The number of consumers for this task in the PARAGRAPH.
  /// @param v          Variadic list of view inputs to workfunction.
  //////////////////////////////////////////////////////////////////////
  template<typename WF, typename... Args>
  void add_task(sched_info_type const& si, std::size_t task_id, WF const& wf,
                std::size_t num_succs, Args const&... args) const
  {
    this->graph().count_task();

    this->graph().add_task(task_id, wf, si, no_preds_t(), num_succs, LQ_LOOKUP,
                           transform_lazy_view<typename std::decay<Args>
                             ::type>:: apply(args)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a new task in the currently running PARAGRAPH.
  ///
  /// @param task_id    User specified identifier that other tasks and
  ///                   @p paragraph_view methods use to refer to this task.
  /// @param wf         The workfunction for this task.
  /// @param preds      vector of predecessor task ids where there is a
  ///                   dependence to enforce but whose return value is not
  ///                   consumed on PARAGRAPH edge.
  /// @param num_succs  The number of consumers for this task in the PARAGRAPH.
  /// @param v          Variadic list of view inputs to workfunction.
  //////////////////////////////////////////////////////////////////////
  template<typename WF, typename... Args>
  void add_task(std::size_t task_id, WF const& wf, pred_list_t preds,
                std::size_t num_succs, Args const&... args) const
  {
    this->graph().count_task();

    this->graph().add_task(
      task_id, wf, none, preds, num_succs, LQ_LOOKUP,
      transform_lazy_view<typename std::decay<Args>::type>::
        apply(args)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a new task in the currently running PARAGRAPH.
  ///
  /// @param sched_info Scheduler metadata (e.g., priority) for task.
  /// @param task_id    User specified identifier that other tasks and
  ///                   @p paragraph_view methods use to refer to this task.
  /// @param wf         The workfunction for this task.
  /// @param preds      vector of predecessor task ids where there is a
  ///                   dependence to enforce but whose return value is not
  ///                   consumed on PARAGRAPH edge.
  /// @param num_succs  The number of consumers for this task in the PARAGRAPH.
  /// @param v          Variadic list of view inputs to workfunction.
  //////////////////////////////////////////////////////////////////////
  template<typename WF, typename... Args>
  void add_task(sched_info_type const& si, std::size_t task_id,
                WF const& wf, pred_list_t preds, std::size_t num_succs,
                Args const&... args) const
  {
    this->graph().count_task();

    this->graph().add_task(
      task_id, wf, si, preds, num_succs, LQ_LOOKUP,
      transform_lazy_view<typename std::decay<Args>::type>::
        apply(args)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a new task in the currently running PARAGRAPH.
  ///
  /// @param wf         The workfunction for this task.
  ///                   dependence to enforce but whose return value is not
  ///                   consumed on PARAGRAPH edge.
  /// @param v          Variadic list of view inputs to workfunction.
  ///
  /// @return PARAGRAPH generated task id guaranteed to be globally unique.
  ///
  /// @remark The created task implicitly has no consumers.
  //////////////////////////////////////////////////////////////////////
  template<typename WF, typename... Args>
  typename std::enable_if<
    !std::is_integral<WF>::value && !std::is_same<WF, sched_info_type>::value,
    void
  >::type
  add_task(WF const& wf, Args const&... args) const
  {
    this->graph().count_task();

    this->graph().add_task(0, wf, none, no_preds_t(), 0, LQ_LOOKUP,
                           transform_lazy_view<typename std::decay<Args>
                             ::type>::apply(args)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Insert a new task in the currently running PARAGRAPH.
  ///
  /// @param sched_info Scheduler metadata (e.g., priority) for task.
  /// @param wf         The workfunction for this task.
  ///                   dependence to enforce but whose return value is not
  ///                   consumed on PARAGRAPH edge.
  /// @param v          Variadic list of view inputs to workfunction.
  ///
  /// @return PARAGRAPH generated task id guaranteed to be globally unique.
  ///
  /// @remark The created task implicitly has no consumers.
  //////////////////////////////////////////////////////////////////////
  template<typename WF, typename... Args>
  typename std::enable_if<!std::is_integral<WF>::value, void>::type
  add_task(sched_info_type const& si, WF const& wf, Args const&... args) const
  {
    this->graph().count_task();

    this->graph().add_task(0, wf, si, no_preds_t(), 0, LQ_LOOKUP,
                           transform_lazy_view<typename std::decay<Args>
                             ::type>::apply(args)...);
  }

  p_object const& no_succs_p_object()
  {
    return this->graph().no_succs_p_object();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Allows the consumer count for a producer task to be specified
  /// independent of its @p add_task invocation.
  ///
  /// @param task_id   Task identifier of task to have consumer count set.
  /// @param num_succs The consumer count for the task.
  ///
  /// @remark The corresponding @p add_task must be passed the defer_spec
  /// constant, denoting that the successor count will be separately specified
  /// by a call to this method.
  //////////////////////////////////////////////////////////////////////
  void set_num_succs(std::size_t task_id, std::size_t num_succs) const
  {
    this->graph().set_num_succs(task_id, num_succs);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the PARAGRAPH task that will be the return value for the
  /// PARAGRAPH from this location.  Can be called by a factory or any task in
  /// the PARAGRAPH with a @p paragraph_view.
  ///
  /// @param task_id Task identifier of task to be used as a result
  //////////////////////////////////////////////////////////////////////
  void set_result(std::size_t task_id) const
  {
    this->graph().set_result(task_id);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief For port flow PARAGRAPHs, connect the output of the given task
  /// to the given index of the output port.
  //////////////////////////////////////////////////////////////////////
  void set_result(std::size_t index, std::size_t task_id) const
  {
    this->graph().set_result(index, task_id);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Specify that a child PARAGRAPH with the given task id
  /// will connect to some of the ports of this parent PARAGRAPH as defined
  /// by the passed mapper.
  /// @see result_container_base::request_parent_flow
  //////////////////////////////////////////////////////////////////////
  template<typename Mapper>
  void set_result_pg(std::size_t task_id, Mapper mapper) const
  {
    this->graph().set_result_pg(task_id, mapper);
  }

  constexpr bool is_local(void) const
  { return true; }
}; // class paragraph_view

} // namespace paragraph_impl


namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Defines return type @p consume signature in single predecessor case.
/// @ingroup pgView
//////////////////////////////////////////////////////////////////////
template<typename T, typename... OptionalFilterParam>
struct consume
{
  using type =
    std::pair<
      ptr_wrapper<
        edge_view<T, typename std::decay<OptionalFilterParam>::type...>>,
      std::size_t>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Defines return type @p consume signature for aggregated, multiple
///   edge consumption and user provided filter function.
/// @ingroup pgView
//////////////////////////////////////////////////////////////////////
template<typename T, typename... OptionalFilterParam>
struct aggregated_consume
{
  using edge_view_t =
    aggregated_edge_view<T, typename std::decay<OptionalFilterParam>::type...>;

  using index_type = typename edge_view_t::index_type;
  using type       = std::pair<ptr_wrapper<edge_view_t>, index_type>;
};

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Creates a parameter passed as a view argument to @p add_task
///   invocations, denoting that the task should consume the value produced
///   by another task.
/// @ingroup pgView
///
/// @tparam The return type of the consumed task.  Not deduced, must be
///         explicitly specified.
///
/// @param pgv    A view of the paragraph the consumption edge is created in.
////
/// @param tid The task identifier of the predecessor task.
///
/// @param filter Optional function object to apply to the predecessor task's
///               result prior to passing it the consumer task's workfunction.
///
/// @return std::pair that implements View/Index concept expected by @p add_task
///
/// Example Usage:
///
/// pgv.add_task(tid, wf, consume<T>(tid)); // wf is unary in this case
///
/// Logically, this should be part of the @ref paragraph_view class.
/// However, since it would then be a member template of class template, the
/// explicit use of the template keyword when calling it would be required
/// (i.e., pgv. template consume<T>(tid)), which is just unacceptable.  Hence,
/// it is a freestanding function.
///
/// The real work of consume (setting up the actual data flow in the PARAGRAPH)
/// is implemented by the @ref edge_container::setup_flow which is called on the
/// @p add_task code path, after some type metafunctions to detect consumption
/// has been requested.
///
/// The motivation behind this filtering (instead of just having the caller
/// wrap their workfunction with another to apply this transformation is that
/// the PARAGRAPH attempts to push the filter to the producing location,
/// applying before value transmission, so that no unneeded bits are
/// communicated.
//////////////////////////////////////////////////////////////////////
template<typename T, typename... OptionalFilterParam>
std::pair<
  ptr_wrapper<edge_view<T, typename std::decay<OptionalFilterParam>::type...>>,
  std::size_t>
consume(paragraph_impl::paragraph_view_base const& pgv,
        std::size_t tid,
        OptionalFilterParam&&... filter)
{
  using res_vw_t =
    edge_view< T, typename std::decay<OptionalFilterParam>::type...>;

  return std::pair<ptr_wrapper<res_vw_t>, std::size_t>(
    ptr_wrapper<res_vw_t>(
      res_vw_t(std::forward<OptionalFilterParam>(filter)...)),
    tid);
}


//////////////////////////////////////////////////////////////////////
/// @brief Sets up consumption as the basic @p consume call, except that
///   multiple predecessors task_ids can be specified.  Their results will be
///   presented to consumer task as a view implementing the array_view concept.
/// @ingroup pgView
///
/// @tparam The return type of the consumed tasks (must be the same).
/// Not deduced, must be explicitly specified.
///
/// @param pgv    A view of the paragraph the consumption edge is created in.
///
/// @param tids   List in a std::vector of task identifiers of predecessors to
///               consume.
///
/// @param filter Optional function object to apply to the predecessor task's
///               result prior to passing it the consumer task's workfunction.
///
/// @return std::pair that implements View/Index concept expected by
/// @p add_task.  The consumer tasks receives an array_view whose elements are
/// the
///
/// Example usage:
///
/// class wf
/// {
///   template<typename View>
///   void operator()(View values) const
///   {
///     x = values[0]; // x assigned result of task 5.
///   }
/// };
///
/// in factory::operator():
///
/// vector<size_t> tids;
/// tids.push_back(5);
/// tids.push_back(3);
/// pgv.add_task(task_id, wf, consume<T>(tids)); // new task has 2 preds.
///
/// @todo Generalize std::vector input parameter to be templated, allowing
/// anything implementing the array_view concept.
//////////////////////////////////////////////////////////////////////
template<typename T, typename TaskIdsRef, typename... OptionalFilterParam>
inline
typename std::enable_if<
  !std::is_integral<typename std::decay<TaskIdsRef>::type>::value,
  typename result_of::aggregated_consume<T, OptionalFilterParam...>::type
>::type
consume(paragraph_impl::paragraph_view_base const& pgv,
        TaskIdsRef&& tids,
        OptionalFilterParam&&... filter)
{
  using result_mf_t =
    typename result_of::aggregated_consume<T, OptionalFilterParam...>;

  using edge_view_t = typename result_mf_t::edge_view_t;
  using ret_t       = typename result_mf_t::type;

  return ret_t(ptr_wrapper<edge_view_t>(
                 edge_view_t(std::forward<OptionalFilterParam>(filter)...)),
               std::forward<TaskIdsRef>(tids));
}


//////////////////////////////////////////////////////////////////////
/// @brief Default mapping function used by port based dataflow protocol
/// to decide where a producer pin is flowed to in the consuming
/// @ref paragraph. Based on the hash-based key mapper of the
/// @ref directory, it's used if no custom mapper is provided by user.
//////////////////////////////////////////////////////////////////////
class default_pin_location_mapper
{
private:
  detail::default_key_mapper<size_t> m_key_mapper;

public:
  std::pair<location_type, loc_qual>
  operator()(size_t key) const
  {
    return m_key_mapper(key);
  }

  bool is_perfect_mapper(void)
  { return true; }

  void set_num_locations(size_t nlocs)
  {
    m_key_mapper.set_num_locations(nlocs);
  }

  void define_type(typer& t)
  {
    t.member(m_key_mapper);
  }
}; // class default_pin_location_mapper


//////////////////////////////////////////////////////////////////////
/// @brief Simple filter to pass to general @ref consume_pg so that
/// all indices in the output port are flowed to the consumer.
//////////////////////////////////////////////////////////////////////
struct unconditional_flow
{
  bool operator()(size_t) const
  { return true; }
};


//////////////////////////////////////////////////////////////////////
/// @brief View class used as input to a nested paragraph created
/// via @p add_task to setup dataflow from a producer, sibling paragraph.
///
/// @tparam T The elementary value type that will be flowed.
/// @tparam OptionalParams Optional parameters to control dataflow.  Currently:
///   (1) PortFilter  - boolean functor queried to see if pin should be flowed.
///   (2) ValueFilter - functor applied to produced value prior to dataflow.
///   (3) PinLocationMapper - maps producer pin to location in this consumer
///     where it should be flowed.
//////////////////////////////////////////////////////////////////////
template<typename T, typename ...OptionalParams>
class nested_pg_view_subview
{
private:
  using param_types =
    typename compute_type_parameters<
      tuple<
        unconditional_flow, detail::df_identity<T>, default_pin_location_mapper
      >,
      OptionalParams...
    >::type;

  using port_filter_t         = typename tuple_element<0, param_types>::type;
  using value_filter_t        = typename tuple_element<1, param_types>::type;
  using pin_location_mapper_t = typename tuple_element<2, param_types>::type;

public:
  using value_type = typename boost::result_of<value_filter_t(T)>::type;

private:
  /// @brief The task id of the producer paragraph in the parent paragraph.
  std::size_t                              m_pred_tid;

  /// @brief Shared pointer to an edge container backing this view, created
  /// lazy in @ref initialize flow during construction of nested paragraph
  /// after transport to enclosing gang.
  mutable std::shared_ptr<edge_container>  m_ct_sptr;

  param_types                              m_params;

  using base_view = edge_view<value_type>;

public:
  using is_pg_port_view = void;

  using reference       = typename base_view::reference;
  /// @todo edge_view has no const_reference type
  using const_reference = typename base_view::reference;
  using index_type      = typename base_view::index_type;
  using gid_type        = index_type;

  /// @todo domain_type is needed for setting dimension of spans in skeletons,
  ///       maybe a better solution is to skip looking at the domain_type of
  ///       the paragraph_view instead of adding type here.
  using domain_type = indexed_domain<value_type>;

  domain_type domain(void) const
  { return domain_type(); }

  nested_pg_view_subview(std::size_t pred_tid, param_types params)
    : m_pred_tid(pred_tid),
      m_params(std::move(params))
  { }

  bool validate(void) const
  { return true; }

  void define_type(typer& t)
  {
    t.member(m_pred_tid);
    t.member(m_params);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called in constructor of nested @ref paragraph.  Create
  /// @ref edge_container to hold in flow from ports. Initiate creation
  /// of edge from producer via the parent paragraph's directory of
  /// nested_pg handles.
  ///
  /// @todo There's an unnecessary forward in the case we fallback to
  /// using an async_rmi to request dataflow initialization via the
  /// parent PARAGRAPH.  This is because we don't know which location in
  /// the parent will be mapped onto this location when it's initialized.
  /// This is available on the source side of the construct() for this
  /// child PARAGRAPH, but there's not a good idiom to disseminate this
  /// in a scalable manner.  The forward is ok, except if global TD elision
  /// is enabled: the semi-randomly chosen parent PARAGRAPH location is not
  /// guaranteed to exist.  Ideally we find a way to ask the runtime to
  /// queue this request for the given handle at the current affinity,
  /// along any location to service it.  At the very least, we may need to
  /// forward enough state information about the parent to this call to
  /// assert global TD hasn't been elided if we fallback to @ref async_rmi.
  //////////////////////////////////////////////////////////////////////
  void initialize_flow(task_graph& tg) const
  {
    pin_location_mapper_t tmp_pin_mapper = get<2>(m_params);

    tmp_pin_mapper.set_num_locations(tg.get_num_locations());

    auto tmp =
      std::make_shared<edge_container>(
        false, &tg, tg.is_persistent(), tmp_pin_mapper);

    m_ct_sptr.swap(tmp);

    auto* parent_pg_ptr = resolve_handle<task_graph>(tg.parent_handle());

    auto f = std::bind(
               &task_graph::child_setup_flow<
                 T, port_filter_t, value_filter_t, pin_location_mapper_t>,
               std::placeholders::_1,
               m_pred_tid,
               tg.task_id(),
               m_ct_sptr->get_rmi_handle(), m_ct_sptr->get_location_id(),
               get<0>(m_params), get<1>(m_params), std::move(tmp_pin_mapper));

    // If the parent PARAGRAPH is initialized on this location, then
    // directly invoke method to request setup of dataflow from a sibling,
    // producer PARAGRAPH to this consumer.
    if (parent_pg_ptr != nullptr)
    {
      task_graph& tg_ref = *parent_pg_ptr;

      gang g(tg_ref);
      f(tg_ref);

      return;
    }

    // else
    //
    // The parent PARAGRAPH doesn't exist at this affinity yet.  Add a message
    // that will be checked for when it is constructed.
    add_pg_message(tg.parent_handle(), std::move(f));
  }

  edge_container& container(void) const
  { return *m_ct_sptr; }

  edge_container& edges(void) const
  { return *m_ct_sptr; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called in consuming paragraph's factory (or another dynamic task)
  /// to initialize out-degree / fan out of each input port for this view.
  //////////////////////////////////////////////////////////////////////
  void set_num_succs(index_type producer_tid, size_t num_succs) const
  {
    m_ct_sptr->set_num_succs(producer_tid, num_succs);
  }

  void assert_perfect_placement(index_type producer_tid) const
  {
    stapl_assert(m_ct_sptr->key_mapper()(producer_tid).first
                   == m_ct_sptr->get_location_id(),
                 "nested_pg_view_subview: imperfect placement detected");
  }
}; // class nested_pg_view_subview


//////////////////////////////////////////////////////////////////////
/// @brief Lightweight view class created by @p consume and passed to
///   @ref add_task to facilitate setup of inter paragraph, port-based
///   dataflow.
///
/// @tparam T The elementary value type that will be flowed.
///
/// @tparam OptionalParams Optional parameters to control dataflow.  Currently:
///   (1) PortFilter  - boolean functor queried to see if pin should be flowed.
///   (2) ValueFilter - functor applied to produced value prior to dataflow.
///   (3) PinLocationMapper - maps producer pin to location in this consumer
///     where it should be flowed.
///
/// @todo Instead of using parent handle for nested locality, implement
///   LQ_DONTCARE behavior.
//////////////////////////////////////////////////////////////////////
template<typename T, typename ...OptionalParams>
class nested_pg_view
{
private:
  using param_types =
    typename compute_type_parameters<
      tuple<
        unconditional_flow, detail::df_identity<T>, default_pin_location_mapper
      >,
      OptionalParams...
    >::type;

  rmi_handle::reference m_parent_pg_handle;
  param_types           m_params;

  //////////////////////////////////////////////////////////////////////
  /// @brief Private constructor which initializes handle as well as
  /// params member. The params member (a tuple), is initialized with either
  /// optionally passed parameters or default constructed objects if the
  /// parameter is either (a) not passed or (b) @ref use_default is passed.
  //////////////////////////////////////////////////////////////////////
  template<typename ...OptionalPassedParams,
           std::size_t ...Indices1, std::size_t ...Indices2>
  nested_pg_view(index_sequence<Indices1...>,
                 index_sequence<Indices2...>,
                 rmi_handle::reference parent_pg_handle,
                 OptionalPassedParams&&... params)
    : m_parent_pg_handle(parent_pg_handle),
      m_params(
        initialize_parameter<
          OptionalPassedParams,
          typename tuple_element<Indices1, param_types>::type
        >::apply(std::forward<OptionalPassedParams>(params))...,
        typename tuple_element<
          sizeof...(OptionalParams) + Indices2, param_types>::type()...)
  { }

public:
  using reference               = nested_pg_view_subview<T, OptionalParams...>;
  using index_type              = std::size_t;
  using task_placement_dontcare = std::true_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor to initialize the object with
  /// the provided optional parameters.  Constructs on index sequence whose
  /// size is the number of undefined (i.e., not provided) optional parameters
  /// and delegates to private constructor to properly initialize @p m_params.
  //////////////////////////////////////////////////////////////////////
  template<typename ...OptionalPassedParams>
  nested_pg_view(rmi_handle::reference parent_pg_handle,
                 OptionalPassedParams&&... params)
    : nested_pg_view(
        make_index_sequence<sizeof...(OptionalParams)>(),
        make_index_sequence<
          tuple_size<param_types>::value - sizeof...(OptionalParams)>(),
        std::move(parent_pg_handle),
        std::forward<OptionalPassedParams>(params)...)
  { }

  rmi_handle::reference nested_locality(size_t) const
  {
    return m_parent_pg_handle;
  }

  reference transporter_reference(index_type idx) const
  {
    return reference(idx, m_params);
  }

  reference operator[](index_type idx) const
  {
    stapl_assert(0, "nested_pg_view::operator[] unexpectedly called");

    return reference(idx, m_params);
  }
}; // class nested_pg_view


//////////////////////////////////////////////////////////////////////
/// @brief Notifier used to wire pin from parent input port to child input
/// port if handle resolution failed (i.e., the parent isn't initialized
/// on the child PG affinity).  Uses child input port's handle, location,
/// and pin id.
//////////////////////////////////////////////////////////////////////
template<typename T>
class parent_pg_notifier
{
private:
  size_t                m_pin_id;
  rmi_handle::reference m_handle;
  location_type         m_location;

  using stored_value_t = typename df_stored_type<T>::type;

public:
  parent_pg_notifier(size_t pin_id,
                     rmi_handle::reference handle,
                     location_type location)
    : m_pin_id(pin_id),
      m_handle(handle),
      m_location(location)
  { }

  void operator()(executor_base&, stored_value_t const& val) const
  {
    async_rmi(m_location, m_handle,
              &edge_container::add_producer<T>, m_pin_id, defer_spec, false);

    async_rmi(m_location, m_handle,
              &edge_container::set_element<T, T const&>, m_pin_id, val);
  }

  void define_type(typer& t)
  {
     t.member(m_pin_id);
     t.member(m_handle);
     t.member(m_location);
  }
}; // struct parent_pg_notifier


//////////////////////////////////////////////////////////////////////
/// @brief View class used as input to a nested paragraph created
/// via @p add_task to setup dataflow from a parent's input port to
/// a child PARAGRAPH.
//////////////////////////////////////////////////////////////////////
template<typename T, typename ...OptionalParams>
class nested_parent_pg_view_subview
{
private:
  std::vector<std::size_t>                 m_producer_pin_ids;
  rmi_handle::reference                    m_parent_ec_handle;

  /// @brief Shared pointer to an edge container backing this view, created
  /// lazy in @ref initialize flow during construction of nested paragraph
  /// after transport to enclosing gang.
  mutable std::shared_ptr<edge_container>  m_ct_sptr;

  using param_types =
    typename compute_type_parameters<
      tuple<identity<size_t>, default_pin_location_mapper>, OptionalParams...
    >::type;

  using pin_location_mapper_type = typename tuple_element<1, param_types>::type;

  // Tuple of Optionally Passed Parameters (or Default Values)
  param_types                              m_params;

  using base_view   = edge_view<T>;

public:
  using is_pg_port_view = void;
  using value_type      = T;
  using reference       = typename base_view::reference;
  using const_reference = typename base_view::reference;
  using index_type      = typename base_view::index_type;
  using gid_type        = index_type;

  // @todo should be removed, added due to need of domain for setting dimension
  //       of span
  using domain_type = indexed_domain<value_type>;

  domain_type domain() const
  {
    return domain_type();
  }

  nested_parent_pg_view_subview(std::vector<std::size_t> producer_pin_ids,
                                rmi_handle::reference parent_ec_handle,
                                param_types params)
    : m_producer_pin_ids(std::move(producer_pin_ids)),
      m_parent_ec_handle(parent_ec_handle),
      m_params(std::move(params))
  { }

  bool validate(void) const
  { return true; }

  void define_type(typer& t)
  {
    t.member(m_producer_pin_ids);
    t.member(m_parent_ec_handle);
    t.member(m_params);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called in constructor of nested @ref paragraph.  Create
  /// @ref edge_container to hold in flow from ports.  If the first location
  /// in nested section, initiate creation of edge from producer via the parent
  /// paragraph's directory of nested_pg handles.
  ///
  /// @todo Right now we iterate over all the passed task_ids to see which are
  /// mapped to this location.  For default pin location mapping, we could
  /// choose just to partition the values (i.e., specialize to avoid iteration).
  ///
  /// @todo Rework pin location mapper parameter to be a factory creating
  /// the mapper given the num locations in the nested section (or find a way
  /// to remove need for num locations parameter).  The set_num_location
  /// interface isn't the best.
  //////////////////////////////////////////////////////////////////////
  void initialize_flow(task_graph& tg) const
  {
    pin_location_mapper_type pin_location_mapper(get<1>(m_params));
    pin_location_mapper.set_num_locations(tg.get_num_locations());

    // Create an edge container to back values flowed to this paragraph
    // from another and facilitate intra-paragraph dataflow.
    auto tmp_ct_ptr =
      std::make_shared<edge_container>(
        false, &tg, tg.is_persistent(), pin_location_mapper);

    m_ct_sptr.swap(tmp_ct_ptr);

    edge_container* parent_ec =
      resolve_handle<edge_container>(m_parent_ec_handle);

    if (parent_ec != nullptr)
    {
      gang g(*parent_ec);

      // Iterate over passed source pin ids.
      // Each location then sets up dataflow via setup_flow() into the parent's
      //  local edge container, if it's initialized, otherwise use an rmi.
      edge_container* ec_ptr = m_ct_sptr.get();

      const auto this_loc = ec_ptr->get_location_id();

      // Iterate over producer side pin_ids
      for (auto producer_pin_id : m_producer_pin_ids)
      {
        // map producer pin id into consumer pin id
        const auto pin_id = get<0>(m_params)(producer_pin_id);

        // if this consumer pin id is mapped to this location, initiate
        // dataflow from the parent input port's edge container via a
        // call to edge_container:setup_flow.
        if (pin_location_mapper(pin_id).first == this_loc)
        {
          // stapl_assert(
          //   !parent_ec->has_perfect_mapper()
          //   || parent_ec->key_mapper()(producer_pin_id).first
          //        == parent_ec->get_location_id(),
          //   "parent mapping unexpectedly imperfect");

          using stored_value_t = typename df_stored_type<T>::type;

          using notifier_t =
            boost::function<void (executor_base&, stored_value_t const&)>;

          parent_ec->setup_flow<T>(
            producer_pin_id,
            notifier_t(
              [pin_id, ec_ptr](executor_base&, stored_value_t const& val)
              {
                ec_ptr->add_producer<T>(pin_id, defer_spec, false);
                ec_ptr->set_element<T, T const&>(pin_id, val);
              }),
            detail::df_identity<T>(), false);
        }
      }

      return;
    }

    // else
    edge_container* ec_ptr = m_ct_sptr.get();
    const auto this_loc = ec_ptr->get_location_id();

    // Iterate over producer side pin_ids
    for (auto producer_pin_id : m_producer_pin_ids)
    {
      // map producer pin id into consumer pin id
      const auto pin_id = get<0>(m_params)(producer_pin_id);

      // If this consumer pin id is mapped to this location, initiate
      // dataflow from the parent input port's edge container via a
      // call to edge_container:setup_flow.
      if (pin_location_mapper(pin_id).first == this_loc)
      {
        add_ec_message(m_parent_ec_handle,
                       std::bind(
                         &edge_container::setup_flow_remote<
                           T, parent_pg_notifier<T>>,
                         std::placeholders::_1,
                         producer_pin_id,
                         parent_pg_notifier<T>(
                           pin_id, ec_ptr->get_rmi_handle(), this_loc)));
      }
    }
  }

  edge_container& container(void) const
  { return *m_ct_sptr; }

  edge_container& edges(void) const
  { return *m_ct_sptr; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called in consuming paragraph's factory (or another dynamic task)
  /// to initialize out-degree / fan out of each input port for this view.
  //////////////////////////////////////////////////////////////////////
  void set_num_succs(index_type producer_tid, size_t num_succs) const
  {
    m_ct_sptr->set_num_succs(producer_tid, num_succs);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Debug method exposed to external code to verify that it's
  /// calling set_num_succs and consuming from the mapped location
  ///
  /// @todo Find a good, not overly conservative condition for call in this
  /// context, refining the commented out call.
  //////////////////////////////////////////////////////////////////////
  void assert_perfect_placement(index_type producer_tid) const
  {
    // stapl_assert(m_ct_sptr->key_mapper()(producer_tid).first
    //                == m_ct_sptr->get_location_id(),
    //              "nested_pg_view_subview: imperfect placement detected");
  }
}; // class nested_parent_pg_view_subview


/////////////////////////////////////////////////////////////////////
/// @brief Lightweight view class created by @p consume and passed to
/// @ref add_task to facilitate setup dataflow from a parent's input port
/// to a child PARAGRAPH.
///
/// @tparam T The elementary value type that will be flowed.
///
/// @tparam OptionalParams Optional parameters to control dataflow.  Currently:
///   (1) PinIdMapper - maps pin identifier in producer port to pin identifier
///     in this consumer.
///   (2) PinLocationMapper - maps producer pin to location in this consumer
///     where it should be flowed.
/////////////////////////////////////////////////////////////////////
template<typename T, typename ...OptionalParams>
class nested_parent_pg_view
{
private:
  using param_types =
    typename compute_type_parameters<
      tuple<identity<size_t>, default_pin_location_mapper>, OptionalParams...
    >::type;

  rmi_handle::reference                    m_parent_ec_handle;

  // Tuple of Optionally Passed Parameters (or Default Values)
  param_types                              m_params;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Private constructor which initializes handle as well as
  /// params member. The params member (a tuple), is initialized with either
  /// optionally passed parameters or default constructed objects if the
  /// parameter is either (a) not passed or (b) @ref use_default is passed.
  //////////////////////////////////////////////////////////////////////
  template<typename ...OptionalPassedParams,
           std::size_t ...Indices1, std::size_t ...Indices2>
  nested_parent_pg_view(index_sequence<Indices1...>,
                        index_sequence<Indices2...>,
                        rmi_handle::reference parent_ec_handle,
                        OptionalPassedParams&&... params)
    : m_parent_ec_handle(parent_ec_handle),
      m_params(
        initialize_parameter<
          OptionalPassedParams,
          typename tuple_element<Indices1, param_types>::type
        >::apply(std::forward<OptionalPassedParams>(params))...,
        typename tuple_element<
          sizeof...(OptionalParams) + Indices2, param_types>::type()...)
  { }

public:
  using reference               =
    nested_parent_pg_view_subview<T, OptionalParams...>;
  using index_type              = std::vector<std::size_t>;
  using task_placement_dontcare = std::true_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor to initialize the object with
  /// the provided optional parameters.  Constructs on index sequence whose
  /// size is the number of undefined (i.e., not provided) optional parameters
  /// and delegates to private constructor to properly initialize @p m_params.
  //////////////////////////////////////////////////////////////////////
  template<typename ...OptionalPassedParams>
  nested_parent_pg_view(rmi_handle::reference parent_ec_handle,
                 OptionalPassedParams&&... params)
    : nested_parent_pg_view(
        make_index_sequence<sizeof...(OptionalParams)>(),
        make_index_sequence<
          tuple_size<param_types>::value - sizeof...(OptionalParams)>(),
        std::move(parent_ec_handle),
        std::forward<OptionalPassedParams>(params)...)
  { }

  rmi_handle::reference nested_locality(size_t) const
  {
    return m_parent_ec_handle;
  }

  reference transporter_reference(index_type idx) const
  {
    return reference(std::move(idx), m_parent_ec_handle, m_params);
  }

  reference operator[](index_type idx) const
  {
    stapl_assert(0, "nested_parent_pg_view::operator[] unexpectedly called");
    return reference(std::move(idx), m_parent_ec_handle, m_params);
  }
}; // class nested_parent_pg_view


//////////////////////////////////////////////////////////////////////
/// @brief Call to allow consumption from a nested paragraph by a sibling
/// paragraph with a filter specified to limit values that are flowed.
/// Called in the parent paragraph's factory.
///
/// @tparam T The elementary value type that will be flowed.
///
/// @tparam OptionalParams Optional parameters to control dataflow.  Currently:
///   (1) PortFilter  - boolean functor queried to see if pin should be flowed.
///   (2) ValueFilter - functor applied to produced value prior to dataflow.
///   (3) PinLocationMapper - maps producer pin to location in this consumer
///     where it should be flowed.
//////////////////////////////////////////////////////////////////////
template<typename T, typename... OptionalParams>
std::pair<ptr_wrapper<nested_pg_view<T, OptionalParams...>>, std::size_t>
consume_pg(paragraph_impl::paragraph_view_base const& pgv,
           std::size_t tid, OptionalParams&&... params)
{
  using view_t = nested_pg_view<T, OptionalParams...>;

  return std::pair<ptr_wrapper<view_t>, std::size_t>(
    view_t(pgv.container().get_rmi_handle(),
           std::forward<OptionalParams>(params)...),
    tid);
}


//////////////////////////////////////////////////////////////////////
/// @brief Signature for task consumption specialized for consumption
/// of a range of values from one of my parent's input ports.
/// Called in the parent paragraph's factory.
///
/// @param view The parent input port to consume from.
/// @param pin_ids The pins from the input parent to consume.
/// @param params Optional parameters.  For now:
///   (1) A pin id mapper that changes the pin_id of the producer
///       when placing it in the consumer input port.
///   (2) A pin location mapper that defines which location in the consumer
///       PARAGRAPH this value will be initially flowed to and managed by.
//////////////////////////////////////////////////////////////////////
template<typename T, typename PinIdsRef,
         typename... OptionalParams1, typename... OptionalParams2>
std::pair<
  ptr_wrapper<
    nested_parent_pg_view<T, typename std::decay<OptionalParams2>::type...>>,
  typename std::decay<PinIdsRef>::type
>
consume_pg(nested_parent_pg_view_subview<T, OptionalParams1...> const& view,
           PinIdsRef&& pin_ids,
           OptionalParams2&&... params)
{
  using index_type = typename std::decay<PinIdsRef>::type;

  using view_type  =
    nested_parent_pg_view<T, typename std::decay<OptionalParams2>::type...>;

  return std::pair<
    ptr_wrapper<view_type>, index_type>(
      view_type(view.container().get_rmi_handle(),
                std::forward<OptionalParams2>(params)...),
      std::forward<PinIdsRef>(pin_ids));
}


//////////////////////////////////////////////////////////////////////
/// @brief Signature for task consumption specialized for consumption
/// of a range of values from the output port of a sibling PARAGRAPH.
///
/// @param view The parent input port to consume from.
/// @param pin_ids The pins from the input parent to consume.
/// @param params Optional parameters.  For now:
///   (1) A pin id mapper that changes the pin_id of the producer
///       when placing it in the consumer input port.
///   (2) A pin location mapper that defines which location in the consumer
///       PARAGRAPH this value will be initially flowed to and managed by.
//////////////////////////////////////////////////////////////////////
template<typename T, typename PinIdsRef,
         typename... OptionalParams1, typename... OptionalParams2>
std::pair<
  ptr_wrapper<
    nested_parent_pg_view<T,  typename std::decay<OptionalParams2>::type...>>,
  typename std::decay<PinIdsRef>::type
>
consume_pg(nested_pg_view_subview<
             T, OptionalParams1...> const& view,
           PinIdsRef&& pin_ids,
           OptionalParams2&&... params)
{
  using index_type = typename std::decay<PinIdsRef>::type;

  using view_type  =
    nested_parent_pg_view<T, typename std::decay<OptionalParams2>::type...>;

  return std::pair<
    ptr_wrapper<view_type>, index_type>(
      view_type(view.container().get_rmi_handle(),
                std::forward<OptionalParams2>(params)...),
      std::forward<PinIdsRef>(pin_ids));
}


//////////////////////////////////////////////////////////////////////
/// @brief Signature for task consumption specialized for consumption from
/// input port (i.e., flow from another paragraph) when a filter is provided
/// which will be applied to all pins prior to transmission to the consuming
/// PARAGRAPH.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Q, typename... OptionalParams>
std::pair<
  ptr_wrapper<pg_edge_view<
    typename nested_pg_view_subview<T, OptionalParams...>::value_type
  >>,
  std::size_t
>
consume(nested_pg_view_subview<Q, OptionalParams...> const& view,
        std::size_t pin_id)
{
  using value_type =
    typename nested_pg_view_subview<Q, OptionalParams...>::value_type;

  static_assert(std::is_same<value_type, T>::value,
                "Invalid filtered flow_type");

  using view_t   = pg_edge_view<value_type>;
  using return_t = std::pair<ptr_wrapper<view_t>, std::size_t>;

  return return_t(ptr_wrapper<view_t>(view_t(view.edges())), pin_id);
}


//////////////////////////////////////////////////////////////////////
/// @brief Signature for task consumption specialized for consumption from
/// an input port backed by one of my parent's input ports.
//////////////////////////////////////////////////////////////////////
template<typename T, typename... OptionalParams>
std::pair<ptr_wrapper<pg_edge_view<T>>, std::size_t>
consume(nested_parent_pg_view_subview<T, OptionalParams...> const& view,
        std::size_t pin_id)
{
  using view_t   = pg_edge_view<T>;
  using return_t = std::pair<ptr_wrapper<view_t>, std::size_t>;

  return return_t(ptr_wrapper<view_t>(view_t(view.edges())), pin_id);
}


//////////////////////////////////////////////////////////////////////
/// @brief Implementation functor of @ref pg_edge_unpackager using
/// partial specialization to call initialize_flow() on objects
/// from instances of the class template nested_pg_view_subview.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct pg_edge_initialize
{
  static void apply(View const&, task_graph&)
  { }
};


template<typename T, typename... OptionalParams>
struct pg_edge_initialize<nested_pg_view_subview<T, OptionalParams...>>
{
  static void
  apply(nested_pg_view_subview<T, OptionalParams...> const& v, task_graph& tg)
  { v.initialize_flow(tg); }
};


template<typename T, typename... OptionalParams>
struct pg_edge_initialize<nested_parent_pg_view_subview<T, OptionalParams...>>
{
  static void
  apply(nested_parent_pg_view_subview<T, OptionalParams...> const& v,
        task_graph& tg)
  { v.initialize_flow(tg); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Called in view initialization of @ref paragraph constructor
/// to initialize inter-paragraph dataflow with producer paragraphs.
//////////////////////////////////////////////////////////////////////
class pg_edge_unpackager
{
private:
  task_graph& m_tg;

public:
  using result_type = void;

  pg_edge_unpackager(task_graph& tg)
    : m_tg(tg)
  { }

  template<typename V>
  void operator()(V& v) const
  {
    pg_edge_initialize<V>::apply(v, m_tg);
  }
};

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_VIEW_HPP

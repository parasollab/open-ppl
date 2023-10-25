/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_ENVIRONMENTS_LOCAL_ENV_HPP
#define STAPL_SKELETONS_ENVIRONMENTS_LOCAL_ENV_HPP

#include "../flows/producer_info.hpp"
#include <memory>
#include <unordered_map>
#include <type_traits>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

template <typename Valuetype>
struct local_entry;

//////////////////////////////////////////////////////////////////////
/// @brief The base class for the entities stored in a @c local_env
/// that can hold heterogeneous types
///
/// @ingroup skeletonsEnvironmentsInternal
//////////////////////////////////////////////////////////////////////
struct local_entry_base
{
  virtual ~local_entry_base()
  { }

  template <typename ValueType>
  ValueType
  get_value()
  {
    return (static_cast<local_entry<ValueType> const&>(*this)).get_value();
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Typed version of @c local_entry that holds the values
/// that are stored in a @c local_env
///
/// @ingroup skeletonsEnvironmentsInternal
//////////////////////////////////////////////////////////////////////
template <typename ValueType>
struct local_entry
  : public local_entry_base
{
  ValueType m_value;

  local_entry(ValueType value)
    : local_entry_base(),
      m_value(value)
  { }

  ValueType get_value() const
  {
    return m_value;
  }

};

} // namespace skeletons_impl

//////////////////////////////////////////////////////////////////////
/// @brief This environment tries to behave as a @c taskgraph_env on a
/// single location. In other words, it runs the workfunctions instead
/// of creating tasks, and stores the values produced by tasks and
/// keeps them as long as they are needed (num_succs is not 0). You
/// should consider this environment as the sequential version of the
/// @c taskgraph_env.
///
/// @see taskgraph_env
/// @ingroup skeletonsEnvironments
///////////////////////////////////////////////////////////////////////
class local_env
{
  typedef std::unordered_map<
            std::size_t, skeletons_impl::local_entry_base*> map_t;
  typedef std::shared_ptr<map_t>                           local_df_t;

  local_df_t m_dep_map;
public:
  local_env()
    : m_dep_map(
        new map_t(),
        [=](map_t* map){
          for (auto&& elem : *map) {
            delete elem.second;
          }
        })
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Runs the workfunction given on the given inputs and stores
  /// the value in the directory of the dependencies.
  ///
  /// @param tid       the unique id assigned for the result of the
  ///                  invocation of the workfunction
  /// @param wf        the workfunction to be executed
  /// @param result_id result_id of this task if it has one
  /// @param num_succs  the exact number of successors for this element.
  ///                  Remember that in some cases this value is set to
  ///                  @c stapl::defer_specs (when the @c spawner is in
  ///                  @c SET_HOLD mode)
  /// @param in...     the producer information for the task's input
  ///                  arguments.
  //////////////////////////////////////////////////////////////////////
  template <bool isResult, typename WF, typename ...In>
  void
  spawn_element(std::size_t tid, int result_id, WF const& wf,
                std::size_t num_succs, In&&... in)
  {
    typedef typename
      boost::result_of<WF(decltype(make_producer(in))...)>::type result_type;

    spawn_element<isResult>(
      typename std::is_same<result_type, void>(),
      tid, std::vector<std::size_t>(), wf, num_succs, std::forward<In>(in)...);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief @copybrief spawn_element
  ///
  /// @param tid      the unique id assigned for the result of the
  ///                 invocation of the workfunction
  /// @param wf       the workfunction to be executed
  /// @param notifications list of the nodes which can proceed as soon as
  ///                 this node is done
  /// @param num_succs the exact number of successors for this element.
  ///                 Remember that in some cases this value is set to
  ///                 @c stapl::defer_specs (when the @c spawner is in
  ///                 @c SET_HOLD mode)
  /// @param in...    the producer information for the task's input
  ///                 arguments.
  //////////////////////////////////////////////////////////////////////
  template <bool isResult, typename WF, typename... In>
  void spawn_element(std::false_type, std::size_t tid, std::size_t result_id,
                     std::vector<std::size_t> const& notifications,
                     WF const& wf, std::size_t num_succs, In&&... in)
  {
    typedef typename
      boost::result_of<WF(decltype(make_producer(in))...)>::type result_type;

    (*m_dep_map)[tid] = new skeletons_impl::local_entry<result_type>(
                              wf(make_producer(std::forward<In>(in))...));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief A specialization of @c spawn_element for the case that
  /// workfunction's result type is void and the notification list is
  /// also provided.
  //////////////////////////////////////////////////////////////////////
  template <bool isResult, typename WF, typename... In>
  void spawn_element(std::true_type, std::size_t tid, std::size_t result_id,
                     std::vector<std::size_t> const& notifications,
                     WF const& wf, std::size_t num_succs, In&&... in)
  {
    wf(make_producer(std::forward<In>(in))...);
  }

  template <typename... Args>
  void pre_spawn(Args&&... args) const
  { }

  template <typename... Args>
  void post_spawn(Args&&... args) const
  { }

  void set_num_succs(std::size_t tid, std::size_t num_succs) const
  { }

  void init_location_info(std::size_t num_PEs,
                          runtime::location_id loc_id)
  { }

  std::size_t get_num_PEs() const
  {
    return 1;
  }

  std::size_t get_PE_id() const
  {
    return 0;
  }

private:
  template <typename Producer>
  typename std::enable_if<
             std::is_same<typename Producer::value_type, void>::value,
             void>::type
  make_producer(Producer const& producer)
  {
    stapl_assert(false, "A producer for a void argument should not be created");
  }

  template <typename Producer>
  typename std::enable_if<
             !(std::is_same<typename Producer::value_type, void>::value),
             typename Producer::value_type>::type
  make_producer(Producer const& producer)
  {
    return (*m_dep_map)[producer.get_index()]->
              template get_value<typename Producer::value_type>();
  }

  template <typename View>
  typename View::reference
  make_producer(flows::view_element_producer<View> const& producer)
  {
    return stapl::detail::make_reference<View>()(producer.get_element(),
                                                 producer.get_index());
  }

  template <typename T, typename A>
  typename flows::view_element_producer<proxy<T, A>>::value_type::value_type
  make_producer(flows::view_element_producer<proxy<T, A>> const& producer)
  {
    return producer.get_element().operator[](producer.get_index());
  }

  template <typename Element>
  Element&
  make_producer(flows::reflexive_producer<Element> const& producer)
  {
    return producer.get_element();
  }
};

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_ENVIRONMENTS_LOCAL_ENV_HPP

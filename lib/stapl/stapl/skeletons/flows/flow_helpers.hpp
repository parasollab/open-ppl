/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FLOWS_FLOW_HELPERS_HPP
#define STAPL_SKELETONS_FLOWS_FLOW_HELPERS_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/pop_front.hpp>
#include <stapl/utility/tuple/pop_back.hpp>
#include <stapl/utility/tuple/front.hpp>
#include <stapl/utility/tuple/back.hpp>

namespace stapl {
namespace skeletons {
namespace flows {

namespace result_of {

template <typename... Flows>
using concat = typename stapl::result_of::tuple_cat<Flows...>::type;

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief Combines two flow tuples
///
/// @param f flow tuples to be concatenated
/// @return a tuple that contains the flows in all the flow tuples
///
/// @ingroup skeletonsFlows
//////////////////////////////////////////////////////////////////////
template <typename... Flows>
flows::result_of::concat<Flows...>
concat(Flows const&... f)
{
  return tuple_cat(f...);
}


namespace result_of {


namespace flows_impl{
template<typename Skeletons, typename F>
struct in_flows_impl;


template<typename... Skeletons, typename F>
struct in_flows_impl<stapl::tuple<Skeletons...>, F>{
  using type = stapl::tuple<
                 typename F::template type<typename Skeletons::in_port_type>...
               >;
};


template<typename Skeletons, typename F>
struct out_flows_impl;


template<typename... Skeletons, typename F>
struct out_flows_impl<stapl::tuple<Skeletons...>, F>
{
  using type = stapl::tuple<
                 typename F::template type<typename Skeletons::out_port_type>...
               >;
};

} // namespace flows_impl


template<typename Skeletons, typename F>
using in_flows = typename flows_impl::in_flows_impl<
                   typename Skeletons::skeletons_type,F>::type;

template<typename Skeletons, typename F>
using out_flows = typename flows_impl::out_flows_impl<
                    typename Skeletons::skeletons, F>::type;

} // namespace result_of

namespace flows_impl {

template<typename F, typename Skeleton, std::size_t... Is>
result_of::in_flows<Skeleton, F>
in_flows_impl(Skeleton const& s, std::size_t lid_offset,
              index_sequence<Is...>)
{
  return stapl::make_tuple(
           F::call(s.template get_skeleton<Is>().in_port(
             s.template lid_offset<Is>(lid_offset)))...);
}

template<typename F, typename Skeleton, std::size_t... Is>
result_of::out_flows<Skeleton, F>
out_flows_impl(Skeleton const& s, std::size_t lid_offset,
               index_sequence<Is...>)
{
  return stapl::make_tuple(
           F::call(s.template get_skeleton<Is>().out_port(
             s.template lid_offset<Is>(lid_offset)))...);
}

} // namespace flows_impl


//////////////////////////////////////////////////////////////////////
/// @brief      A flow modifier that extracts the i'th flow from a
/// flow tuple.
///
/// @tparam I   The index of the flow to extract. If -1, then the back
/// is used.
//////////////////////////////////////////////////////////////////////
template<int I>
struct get_flow_modifier
{
  template<class FlowTuple>
  using type = typename stapl::tuple_element<I, FlowTuple>::type;

  template<class FlowTuple>
  static type<FlowTuple> call(FlowTuple const& flows)
  {
    return std::get<I>(flows);
  }

};


template<>
struct get_flow_modifier<-1>
{
  template<typename FlowTuple>
  using type = typename tuple_ops::result_of::back<FlowTuple>::type;

  template<class FlowTuple>
  static type<FlowTuple> call(FlowTuple const& flows)
  {
    return tuple_ops::back(flows);
  }
};

using get_front_modifier = get_flow_modifier<0>;
using get_back_modifier = get_flow_modifier<-1>;


//////////////////////////////////////////////////////////////////////
/// @brief Returns a flow tuple created from the input flows of
/// all the skeletons received in @c skeletons.
///
/// @param skeletons  a list of skeletons for which the in-flows are
///                  obtained
/// @tparam F        which flow modifier to apply on each in-flow
/// @return a flow tuple containing all the input flows of the given
///         skeleton set
///
/// @ingroup skeletonsFlows
//////////////////////////////////////////////////////////////////////
template <typename F, typename Skeleton>
flows::result_of::in_flows<Skeleton, F>
in_flows(Skeleton skeleton, std::size_t lid_offset)
{
  return flows_impl::in_flows_impl<F>(skeleton, lid_offset,
           make_index_sequence<Skeleton::skeletons_size::value>());
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns a flow tuple created from the output flows of
/// all the skeletons received in @c skeletons.
///
/// @param  skeletons a list of skeletons for which the out_flows are
///                  obtained
/// @tparam F        which flow modifier to apply on each out-flow
/// @return a flow tuple containing all the input flows of the given
///         skeleton set
///
/// @ingroup skeletonsFlows
//////////////////////////////////////////////////////////////////////
template <typename F, typename Skeleton>
flows::result_of::out_flows<Skeleton, F>
out_flows(Skeleton skeleton, std::size_t lid_offset)
{
  return flows_impl::out_flows_impl<F>(skeleton, lid_offset,
           make_index_sequence<Skeleton::skeletons_size::value>());
}

} // namespace flows
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FLOWS_FLOW_HELPERS_HPP

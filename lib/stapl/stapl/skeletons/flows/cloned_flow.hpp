/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_CLONED_FLOW_HPP
#define STAPL_SKELETONS_CLONED_FLOW_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/flows/numbered_flow.hpp>

namespace stapl {
namespace skeletons {
namespace flows {

namespace result_of {

template <typename Flow, std::size_t N,
          typename Indices = stapl::make_index_sequence<N>>
struct cloned_flow;

template <typename Flow, std::size_t N, std::size_t...Indices>
struct cloned_flow<Flow, N, stapl::index_sequence<Indices...>>
{
  using type = stapl::tuple<numbered_flow<Indices, Flow>...>;
};

} // namespace result_of

namespace flows_impl {

template <typename Flow, std::size_t... Indices>
auto
cloned_flow(Flow const& flow, stapl::index_sequence<Indices...>&&)
STAPL_AUTO_RETURN((
  stapl::make_tuple(make_numbered_flow<Indices>(flow)...)
))

} // namespace flows_impl

//////////////////////////////////////////////////////////////////////
/// @brief Clones a given flow for the number of times specified
/// by @c N. A cloned flow created by this method will wrap the given
/// flow by @c numbered_flow with increasing indices starting from 0.
///
/// @tparam N   the number of times the flow should be cloned
/// @param flow a single flow to be cloned
///
/// @ingroup skeletonsFlows
//////////////////////////////////////////////////////////////////////
template <std::size_t N, typename Flow>
typename flows::result_of::cloned_flow<Flow, N>::type
cloned_flow(Flow&& flow)
{
  return flows_impl::cloned_flow(
           std::forward<Flow>(flow),
           stapl::make_index_sequence<N>());
}

} // namespace flows
} // namespace skeletons
} // namespace stapl
#endif // STAPL_SKELETONS_CLONED_FLOW_HPP

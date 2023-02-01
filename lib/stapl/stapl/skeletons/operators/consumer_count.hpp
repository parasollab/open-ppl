/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_OPERATORS_CONSUMER_COUNT_HPP
#define STAPL_SKELETONS_OPERATORS_CONSUMER_COUNT_HPP

#include <stapl/utility/integer_sequence.hpp>
#include <initializer_list>
#include <numeric>

namespace stapl {
namespace skeletons {

template <typename OutFlow, typename Coord>
std::size_t consumer_count(OutFlow const& out, Coord const& coord);

template <typename... OutFlow, typename Coord>
std::size_t consumer_count(tuple<OutFlow...> const& out, Coord const& coord);

namespace skeletons_impl {

template <typename OutFlow, typename Coord, std::size_t... Indices>
std::size_t consumer_count(OutFlow const& out, Coord const& coord,
                           index_sequence<Indices...>&&)
{
  std::initializer_list<std::size_t> c = {
    stapl::skeletons::consumer_count(stapl::get<Indices>(out), coord)...};
  return std::accumulate(c.begin(), c.end(), std::size_t{0});
}

} // namespace skeletons_impl

////////////////////////////////////////////////////////////////////
/// @brief consumer count overload for the case that we just want
///        to know the number of consumers just for a @c FlowIndex flow
////////////////////////////////////////////////////////////////////
template <std::size_t FlowIndex, typename... OutFlow, typename Coord>
std::size_t consumer_count(tuple<OutFlow...> const& out, Coord const& coord)
{
  return stapl::get<FlowIndex>(out).consumer_count(coord);
}

//////////////////////////////////////////////////////////////////////
/// @brief Computes the total number of consumers for a given coordinate
/// on a single flow.
///
/// @param out   a flow (non-tuple)
/// @param coord the coordinate for which the number of consumers is
///              calculated
///
/// @return total number of consumers of a data flow node at a given
///         @c coord
//////////////////////////////////////////////////////////////////////
template <typename OutFlow, typename Coord>
std::size_t
consumer_count(OutFlow const& out, Coord const& coord)
{
  return out.consumer_count(coord);
}

//////////////////////////////////////////////////////////////////////
/// @brief Computes the total number of consumers for a given coordinate
/// on a given tuple of flows.
///
/// @param out   a tuple of flows
/// @param coord the coordinate for which the number of consumers is
///              calculated
///
/// @return total number of consumers of a data flow node at a given
///         @c coord
//////////////////////////////////////////////////////////////////////
template <typename... OutFlow, typename Coord>
std::size_t
consumer_count(std::tuple<OutFlow...> const& out, Coord const& coord)
{
  return skeletons_impl::consumer_count(
    out, coord, make_index_sequence<sizeof...(OutFlow)>());
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_OPERATORS_CONSUMER_COUNT_HPP

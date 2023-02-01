/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_OPERATORS_ELEM_HPP
#define STAPL_SKELETONS_OPERATORS_ELEM_HPP

#include <type_traits>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/flows/elem_flow.hpp>
#include <stapl/skeletons/spans/balanced.hpp>
#include <stapl/skeletons/spans/blocked.hpp>
#include "elem_impl.hpp"

namespace stapl {
namespace skeletons {
namespace result_of {

template <typename Span, typename PD,
          bool nested = is_nested_skeleton<typename PD::op_type>::value>
struct span_default_type;

template <typename Span, typename PD>
struct span_default_type<Span, PD, false>
{
  using type =
    stapl::default_type<Span, spans::balanced<>>;
};

template <typename Span, typename PD>
struct span_default_type<Span, PD, true>
{
  using skeleton_t =
    skeletons_impl::elem<PD, stapl::default_type<Span, spans::balanced<>>,
                         flows::elem_f::doall>;

  static constexpr size_t num_level = Traverse<skeleton_t>::type::value;
  static constexpr size_t dims      = Traverse<skeleton_t>::dims::value;
  using type = spans::nest_blocked<Span::dims_num::value, dims,
                                   typename PD::op_type, num_level>;
};

template <typename Span, typename Flows, typename PD>
using elem = skeletons_impl::elem<
  typename std::decay<PD>::type,
  typename span_default_type<typename std::decay<Span>::type,
                             typename std::decay<PD>::type>::type,
  stapl::default_type<Flows, flows::elem_f::doall>>;

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief An elementary is an operator that converts parametric
/// dependencies to skeletons. It wraps a parametric dependency with
/// @c Flows and @c Span information.
///
/// @tparam Span  the iteration space for elements in this elementary.
///               The default span is @c balanced
/// @tparam Flows the flow to be used for the elementary skeleton.
///               The default flow is a @c forked flow
/// @param  pd    the parametric dependency to be wrapped
/// @return an elementary skeleton
///
/// @see flows::elem_f::forked
/// @see spans::balanced
///
/// @ingroup skeletonsOperators
//////////////////////////////////////////////////////////////////////
template <typename Span  = stapl::use_default,
          typename Flows = stapl::use_default,
          typename PD>
result_of::elem<Span, Flows, PD>
elem(PD&& pd, Span&& span)
{
  return result_of::elem<Span, Flows, PD>(
           std::forward<PD>(pd), std::forward<Span>(span));
}

//////////////////////////////////////////////////////////////////////
/// @brief  An overload when the span instance is not passed
/// @tparam Span  the iteration space for elements in this elementary.
///               The default span is @c balanced
/// @tparam Flows the flow to be used for the elementary skeleton.
///               The default flow is a @c forked flow
/// @param  pd    the parametric dependency to be wrapped
/// @return an elementary skeleton
///
/// @see flows::elem_f::forked
/// @see spans::balanced
///
/// @ingroup skeletonsOperators
//////////////////////////////////////////////////////////////////////
template <typename Span  = stapl::use_default,
          typename Flows = stapl::use_default,
          typename PD>
result_of::elem<Span, Flows, PD>
elem(PD&& pd)
{
  return result_of::elem<Span, Flows, PD>(std::forward<PD>(pd));
}


} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_OPERATORS_ELEM_HPP

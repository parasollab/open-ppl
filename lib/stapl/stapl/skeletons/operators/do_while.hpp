/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_OPERATORS_DO_WHILE_HPP
#define STAPL_SKELETONS_OPERATORS_DO_WHILE_HPP

#include <type_traits>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/spans/per_location.hpp>
#include <stapl/skeletons/flows/do_while_flows.hpp>
#include <stapl/skeletons/param_deps/do_while_pd.hpp>
#include "do_while_impl.hpp"

namespace stapl {
namespace skeletons {
namespace result_of {

template <typename Flows,
          typename Body, typename Reduce, typename ContCond>
using do_while = skeletons_impl::do_while<
                   typename std::decay<Body>::type,
                   typename std::decay<Reduce>::type,
                   typename result_of::elem<
                     spans::per_location,
                     stapl::use_default,
                     skeletons_impl::do_while_pd<
                       typename std::decay<ContCond>::type>>,
                   stapl::default_type<Flows, flows::do_while_flows::piped>>;

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief A do while skeleton is used when a loop with dynamically
/// determined condition is required. After each iteration the loop
/// condition provided by @c cont_cond is evaluated and loop
/// will continue or stop based on the input to the cont_cond.
///
/// @tparam Flows           the flow to be used for the do-while. By
///                         default, a @c do_while_flows::piped is used.
/// @param  body_skeleton   the body skeleton is the body of each iteration
/// @param  reduce_skeleton the reduction skeleton is used to compute the
///                         condition of the loop
/// @param  cont_cond         the condition functor to be used in order to
///                         determine if the loop continues or stops
/// @return a do while skeleton
///
/// @see do_while_pd
///
/// @ingroup skeletonsOperators
//////////////////////////////////////////////////////////////////////
template <typename Flows = stapl::use_default,
          typename Body, typename Reduce, typename ContCond>
result_of::do_while<Flows, Body, Reduce, ContCond>
do_while(Body&& body_skeleton, Reduce&& reduce_skeleton,
         ContCond&& cont_cond)
{
  return result_of::do_while<Flows, Body, Reduce, ContCond>(
           std::forward<Body>(body_skeleton),
           std::forward<Reduce>(reduce_skeleton),
           skeletons::elem<spans::per_location>(
             do_while_pd(std::forward<ContCond>(cont_cond))));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_OPERATORS_DO_WHILE_HPP

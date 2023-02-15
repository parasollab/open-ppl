/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_SINK_HPP
#define STAPL_SKELETONS_FUNCTIONAL_SINK_HPP

#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/flows/compose_flows.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include "copy.hpp"

namespace stapl {
namespace skeletons {
namespace result_of {

template <typename ValueType,
          typename SrcSkeleton, typename DestSkeleton>
using sink = decltype(
               skeletons::compose<flows::compose_flows::last_input_to_all>(
                 std::declval<SrcSkeleton>(),
                 std::declval<DestSkeleton>()));

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief This sink skeleton assumes a default span for the created
/// skeleton
///
/// @tparam ValueType    the type of elements to be copied
/// @param  skeleton      the skeleton to read the input from
/// @param  dest_skeleton a customized sink skeleton. By default this
///                      is assumed to be a copy skeleton
/// @return a sink skeleton with a customized destination skeleton
///
/// @see copy
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename ValueType,
          typename Span = stapl::use_default,
          typename SrcSkeleton,
          typename DestSkeleton =
            decltype(skeletons::copy<ValueType>(skeleton_traits<Span>()))>
result_of::sink<ValueType, SrcSkeleton, DestSkeleton>
sink(SrcSkeleton&& skeleton,
     DestSkeleton&& dest_skeleton =
       skeletons::copy<ValueType>(skeleton_traits<Span>()))
{
  using namespace flows;
  return skeletons::compose<compose_flows::last_input_to_all>(
           std::forward<SrcSkeleton>(skeleton),
           std::forward<DestSkeleton>(dest_skeleton));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_SINK_HPP

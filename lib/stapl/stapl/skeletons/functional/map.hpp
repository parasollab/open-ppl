/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_MAP_HPP
#define STAPL_SKELETONS_FUNCTIONAL_MAP_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/functional/skeleton_traits.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/transformations/transform.hpp>
#include "zip.hpp"

namespace stapl {
namespace skeletons {
namespace result_of {

template <typename Op, typename SkeletonTraits>
using map = result_of::zip<1, Op, SkeletonTraits>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief A filtered map is similar to @c map skeleton but it applies
/// a filter function on the producer side before sending data along
/// the edges to each parametric dependency.
///
/// @tparam Span   the iteration space for the elements in this
///                skeleton
/// @tparam Flows  the flow to be used for this skeleton
/// @param  op     the workfunction to be used in each map parametric
///                skeleton.
/// @param  filter the filter function to be used on the producer side
///                before sending data to a parametric dependency
/// @return a map skeleton with a filter on the incoming edges
///
/// @see map
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename Op,
          typename SkeletonTraits = skeletons_impl::default_skeleton_traits,
          typename =
            typename std::enable_if<
              !is_skeleton<typename std::decay<Op>::type>::value>::type>
result_of::map<Op, SkeletonTraits>
map(Op&& op, SkeletonTraits&& traits = SkeletonTraits())
{
  return result_of::map<Op, SkeletonTraits>(
           std::forward<Op>(op),
           std::forward<SkeletonTraits>(traits));
}

//////////////////////////////////////////////////////////////////////
/// @brief Creates a nested skeleton composition by transforming the
/// inner skeleton to a suitable skeleton for nested execution.
///
/// @tparam ExecutionParams the execution parameters for the nested section
/// @param  op              the skeleton to be used in the nested section
/// @param  traits          the traits to be used
///                         (default = default_skeleton_traits)
/// @see map
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename Op,
          typename SkeletonTraits = skeletons_impl::default_skeleton_traits,
          typename ExecutionParams = skeletons_impl::default_execution_params,
          typename = typename std::enable_if<
            is_skeleton<typename std::decay<Op>::type>::value>::type>
result_of::map<decltype(skeletons::transform<tags::nest>(
                 std::declval<Op>(), std::declval<ExecutionParams>())),
               SkeletonTraits>
map(Op&& op,
    SkeletonTraits&& traits = SkeletonTraits(),
    ExecutionParams&& execution_params = ExecutionParams())
{
  return skeletons::map(
           transform<tags::nest>(
             std::forward<Op>(op),
             std::forward<ExecutionParams>(execution_params)),
           std::forward<SkeletonTraits>(traits));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_MAP_HPP

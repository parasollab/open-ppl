/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_COARSE_REDUCE_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_COARSE_REDUCE_HPP

#include <stapl/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/transformations/transform.hpp>

#include <stapl/skeletons/transformations/optimizers/reduce.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/functional/map.hpp>

namespace stapl {
namespace skeletons {
namespace transformations {


template <typename S, typename SkeletonTag, typename CoarseTag>
struct transform;


//////////////////////////////////////////////////////////////////////
/// @brief A simple coarse-grain reduce can be created by first
/// computing local reduction on each location and applying reduction
/// on the outcome of those local reductions.
///
/// @tparam S            the reduce skeleton to be coarsened
/// @tparam Tag          type of the reduce skeleton passed in
/// @tparam CoarseTag    a tag to specify the required specialization for
///                      coarsening
/// @tparam ExecutionTag a tag to specify the execution method used for
///                      the coarsened chunks
///
/// @see skeletonsTagsCoarse
/// @see skeletonsTagsExecution
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template<typename S, typename Tag, typename CoarseTag, typename ExecutionTag>
struct transform<S, tags::reduce<Tag>,
                 tags::coarse<CoarseTag, ExecutionTag>>
{
  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::compose(
      skeletons::map(skeletons::wrap<ExecutionTag>(skeleton)),
      skeleton)
  ))
};

} // namespace transformations
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_COARSE_REDUCE_HPP

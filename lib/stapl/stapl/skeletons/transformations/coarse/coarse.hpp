/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_COARSE_COARSE_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_COARSE_COARSE_HPP

#include <type_traits>
#include <stapl/utility/utility.hpp>
#include <stapl/utility/use_default.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/transformations/transform.hpp>

namespace stapl {
namespace skeletons {

//////////////////////////////////////////////////////////////////////
/// @brief An indirect call to a specialized coarsening for a specific
/// skeleton.
///
/// When @c coarse is called over a skeleton, this method redirects the
/// call to related implementation for a coarsened execution of that
/// skeleton. This indirection can be used later on for automatic
/// coarsening.
///
/// @param skeleton     the skeleton to be coarsened
/// @param CoarseTag    a tag to specify the required specialization for
///                     coarsening
/// @param ExecutionTag a tag to specify the execution method used for
///                     the coarsened chunks
///
/// @return a coarsened version of the given skeleton
///
/// @see skeletonsTagsCoarse
/// @see skeletonsTagsExecution
///
/// @ingroup skeletonsTransformations
//////////////////////////////////////////////////////////////////////
template <typename CoarseTag    = stapl::use_default,
          typename ExecutionTag = tags::sequential_execution,
          typename S,
          typename =
            typename std::enable_if<
              is_skeleton<typename std::decay<S>::type>::value>::type>
auto
coarse(S&& skeleton)
STAPL_AUTO_RETURN((
  skeletons::transform<tags::coarse<CoarseTag, ExecutionTag>>(
    std::forward<S>(skeleton))
))

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_COARSE_COARSE_HPP

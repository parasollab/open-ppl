/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_COARSE_ZIP_REDUCE_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_COARSE_ZIP_REDUCE_HPP

#include <type_traits>
#include <stapl/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/transformations/transform.hpp>

#include <stapl/skeletons/transformations/optimizers/zip_reduce.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/functional/reduce.hpp>
#include <stapl/skeletons/operators/compose.hpp>

namespace stapl {
namespace skeletons {
namespace transformations {

template <typename S, typename SkeletonTag, typename CoarseTag>
struct transform;


//////////////////////////////////////////////////////////////////////
/// @brief A coarse-grain zip_reduce can be created by sequencing a
/// coarse-grain @c zip of @c zip_reduces with a @c reduce.
///
/// Please notice that we are using @c optimizers::zip_reduce_optimizer
/// by default.
///
/// @tparam S            the @c zip_reduce skeleton to be coarsened
/// @tparam arity        number of inputs to the given zip_reduce skeleton
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
template<typename S, int arity, typename CoarseTag, typename ExecutionTag>
struct transform<S, tags::zip_reduce<arity>,
                 tags::coarse<CoarseTag, ExecutionTag>>
{
public:
  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::compose(
      skeletons::zip<arity>(
        skeletons::wrap<
          ExecutionTag,
          std::is_base_of<dynamic_wf, typename S::zip_op_type>::value
        >(skeleton)),
      skeletons::reduce(skeleton.get_reduce_op()))
  ))
};

} // namespace transformations
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_COARSE_ZIP_REDUCE_HPP

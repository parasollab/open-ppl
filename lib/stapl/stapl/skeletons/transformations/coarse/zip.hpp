/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_COARSE_ZIP_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_COARSE_ZIP_HPP


#include <type_traits>
#include <stapl/skeletons/transformations/coarse/coarse.hpp>
#include <stapl/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/transformations/transform.hpp>

#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/transformations/optimizers/zip.hpp>

namespace stapl {
namespace skeletons {
namespace transformations {

template <typename S, typename SkeletonTag, typename CoarseTag>
struct transform;

//////////////////////////////////////////////////////////////////////
/// @brief A simple coarse-grain zip can be created by creating a
/// @c zip of @c zips. This is the perfect example of simple division
/// of work.
///
/// @tparam S            the zip skeleton to be coarsened
/// @tparam Tag          identifier of the given zip skeleton
/// @tparam arity        number of inputs to the given zip skeleton
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
template<typename S, typename Tag, int arity,
         typename CoarseTag, typename ExecutionTag>
struct transform<S, tags::zip<Tag, arity>,
                 tags::coarse<CoarseTag, ExecutionTag>>
{
  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::zip<arity>(
      skeletons::wrap<
        ExecutionTag,
        std::is_base_of<dynamic_wf, typename S::op_type>::value
      >(skeleton),
      skeleton_traits<typename S::span_type, S::set_result>())
  ))
};

} // namespace transformations
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_COARSE_ZIP_HPP

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_COARSE_WAVEFRONT_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_COARSE_WAVEFRONT_HPP

#include <stapl/skeletons/transformations/coarse/coarse.hpp>
#include <stapl/skeletons/functional/wavefront.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/transformations/optimizers/wavefront.hpp>
#include <stapl/skeletons/utility/position.hpp>

namespace stapl {
namespace skeletons {
namespace transformations {


//////////////////////////////////////////////////////////////////////
/// @brief A coarse-grain @c wavefront skeleton on multiarrays is expressed
/// as a wavefront of wavefronts (wavefront(wavefront(op)), in which @c op
/// is the user-specified fine-grain operation).
///
/// In the default case, the nested wavefront skeleton is executed as
/// by creating a paragraph in the nested section. In default case, the full
/// domain of wavefront is represented as the output of computation, which later
/// will be filtered based on the filtered passed to the skeleton.
///
/// @tparam S            the wavefront skeleton to be coarsened
/// @tparam i            the dimensionality of the given wavefront skeleton
/// @tparam CoarseTag    a tag to specify the required specialization for
///                      coarsening
/// @tparam ExecutionTag a tag to specify the type of execution for
///                      wrapped skeleton(sequential, default...)
///
/// @see skeletonsTagsCoarse
/// @see skeletonsTagsExecution
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <typename S, int i, typename ExecutionTag>
struct transform<S, tags::wavefront<i>, tags::coarse<use_default, ExecutionTag>>
{
  static auto call(S const& skeleton)
    STAPL_AUTO_RETURN((
      skeletons::wavefront<S::number_of_inputs>(
        skeletons::wrap<ExecutionTag>(skeleton),
        skeleton.get_start_corner(),
        skeletons::skeleton_traits<typename S::span_type, S::set_result>(
          wavefront_utils::wavefront_filter<
            S::number_of_dimensions, typename S::filter_type
          >(skeleton.get_start_corner(), skeleton.get_filter())))))
};


//////////////////////////////////////////////////////////////////////
/// @brief A coarse-grain @c wavefront skeleton on multiarrays is expressed
/// as a wavefront of wavefronts (wavefront(wavefront(op)), in which @c op
/// is the user-specified fine-grain operation).
///
/// This specialization computes only boundary planes of the wavefront, rather
/// than whole of the domain.
///
/// @tparam S            the wavefront skeleton to be coarsened
/// @tparam i            the dimensionality of the given wavefront skeleton
/// @tparam CoarseTag    a tag to specify the required specialization for
///                      coarsening
/// @tparam ExecutionTag a tag to specify the type of execution for
///                      wrapped skeleton(sequential, default...)
///
/// @see skeletonsTagsCoarse
/// @see skeletonsTagsExecution
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <typename S, int i, typename ExecutionTag>
struct transform<
  S, tags::wavefront<i>, tags::coarse<tags::only_boundary, ExecutionTag>>
{
  static auto call(S const& skeleton)
    STAPL_AUTO_RETURN((
      skeletons::wavefront<S::number_of_inputs>(
        skeletons::wrap<ExecutionTag>(skeleton),
        skeleton.get_start_corner(),
        skeletons::skeleton_traits<typename S::span_type, S::set_result>(
          wavefront_utils::wavefront_filter_v2<
            S::number_of_dimensions, typename S::filter_type
          >(skeleton.get_start_corner(), skeleton.get_filter())))))
};


} // namespace transformations
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_COARSE_WAVEFRONT_hpp

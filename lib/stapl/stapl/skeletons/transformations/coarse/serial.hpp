/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_COARSE_SERIAL_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_COARSE_SERIAL_HPP

#include <type_traits>
#include <stapl/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/transformations/transform.hpp>

#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/functional/serial.hpp>
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
/// @tparam S            serial skeleton to be coarsened
/// @tparam arity        number of inputs in the serial skeleton
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
struct transform<S, tags::serial<arity>,
                 tags::coarse<CoarseTag, ExecutionTag>>
{
  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::serial<arity>(
      skeletons::wrap<ExecutionTag>(skeletons::zip<arity>(skeleton.get_op())),
      skeleton.get_number_of_sets())
  ))
};

} // namespace transformations
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_COARSE_SERIAL_HPP

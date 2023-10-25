/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_NESTED_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_NESTED_HPP

#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/transformations/optimizer.hpp>

namespace stapl {
namespace skeletons {
namespace optimizers {

template <typename SkeletonTag, typename ExecutionTag>
struct optimizer;

//////////////////////////////////////////////////////////////////////
/// @brief A @c nested optimizer will spawn the skeleton in parallel
/// allowing nested parallelism to be defined and exploited in a high
/// level.
///
/// @see skeletons::execute
///
/// @ingroup skeletonsTransformationsNest
//////////////////////////////////////////////////////////////////////
template <typename SkeletonTag, typename ExecutionTag>
struct optimizer;


template <typename Tag>
struct optimizer<Tag, tags::nested_execution<false>>
{
  // would be discarded if a deriving optimizer redefines result_type
  template <typename R>
  struct result;

  template <typename Optimizer, typename OutputValueType>
  struct result<Optimizer(OutputValueType)>
  {
    using traits_t = stapl::skeletons::skeleton_execution_traits<
                       Tag, tags::nested_execution<false>>;
    using type = typename traits_t::template result_type<OutputValueType>;
  };
};

//////////////////////////////////////////////////////////////////////
/// @brief An @c optimizer with the nested execution strategy, spawns
/// the given skeleton (creates a data flow graph).
///
/// The value 'true' in default_execution denotes that the given
/// skeleton is reducing the input to a single value. In order to
/// provide the reduced value to all the participating locations,
/// the given skeleton needs to be composed with a @c broadcast_to_locs
/// skeleton, and should then be wrapped by a @c sink_value skeleton.
///
/// @see skeletons::execute
///
/// @ingroup skeletonsTransformationsNest
//////////////////////////////////////////////////////////////////////
template <typename Tag>
struct optimizer<Tag, tags::nested_execution<true>>
{
  template <typename FlowValueType>
  struct result;

  template <typename Optimizer, typename FlowValueType>
  struct result<Optimizer(FlowValueType)>
  {
    using type = FlowValueType;
  };
};


} // namespace optimizers
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_OPTIMIZERS_NESTED_HPP

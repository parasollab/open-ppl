/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_COARSE_SCAN_REDUCE_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_COARSE_SCAN_REDUCE_HPP

#include <type_traits>
#include <stapl/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/transformations/transform.hpp>

#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/transformations/coarse/coarse.hpp>
#include <stapl/skeletons/transformations/optimizers/scan.hpp>
#include <stapl/skeletons/transformations/optimizers/reduce.hpp>

#include <stapl/algorithms/identity_value.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/transformations/coarse/scan.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/functional/reduce.hpp>
#include <stapl/skeletons/functional/map.hpp>
#include <stapl/skeletons/functional/scan.hpp>
#include <stapl/skeletons/functional/scan_reduce.hpp>
#include <stapl/skeletons/functional/pre_broadcast.cpp>
#include <stapl/skeletons/param_deps/shifted_first_pd.hpp>
#include <stapl/skeletons/param_deps/zip_pd.hpp>
#include <stapl/skeletons/flows/inline_flows.hpp>
#include <stapl/skeletons/spans/only_nearest_pow_two.hpp>

namespace stapl {
namespace skeletons {
namespace transformations {


template <typename S, typename SkeletonTag, typename CoarseTag>
struct transform;

namespace ph = stapl::skeletons::flows::inline_flows::placeholders;

//////////////////////////////////////////////////////////////////////
/// @brief Coarsened version of exclusive scan_reduce skeleton
///
/// @tparam S            the scan skeleton
/// @tparam Tag          type of exclusive scan to use for the conquering phase
/// @tparam CoarseTag    a tag to specify the required specialization for
///                      coarsening
/// @tparam ExecutionTag a tag to specify the execution method used for
///                      the coarsened chunks
///
/// @see wrapped_skeleton
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <typename S, typename Tag,
          typename CoarseTag, typename ExecutionTag>
struct transform<S, tags::scan_reduce<Tag, tags::exclusive>,
                 tags::coarse<CoarseTag, ExecutionTag>>
{
  using value_t = typename S::value_t;

  static auto call(S const& skeleton)
    STAPL_AUTO_RETURN((compose<skeletons::tags::inline_flow>(
      ph::x<0>() << map(
        skeletons::wrap<ExecutionTag>(skeletons::reduce(skeleton.get_op()))) |
        ph::input<0>(),
      ph::x<1>() << skeletons::scan<Tag>(skeleton.get_op(),
                                         skeleton.get_init_value()) |
        ph::x<0>(),
      ph::x<2>() << skeletons::elem(
        skeletons::scan_helpers::find_scan_update_phase_pd::
          call<tags::exclusive, stapl::use_default>(skeleton.get_op())) |
        (ph::x<1>(), ph::input<0>(), ph::input<1>()),
      ph::x<3>() << skeletons::pre_broadcast<2>(
        skeleton.get_op(), skeleton_traits<spans::only_nearest_pow_two>()) |
        (ph::x<0>(), ph::x<1>()),
      ph::x<4>() << skeletons::broadcast_to_locs<true, tags::right_aligned>() |
        ph::x<3>())))
};

//////////////////////////////////////////////////////////////////////
/// @brief Coarsened version of inclusive scan_reduce skeleton
///
/// @tparam S            the scan skeleton
/// @tparam Tag          type of inclusive scan to use for the conquering phase
/// @tparam CoarseTag    a tag to specify the required specialization for
///                      coarsening
/// @tparam ExecutionTag a tag to specify the execution method used for
///                      the coarsened chunks
///
/// @see wrapped_skeleton
///
/// @ingroup skeletonsTransformationsCoarse
//////////////////////////////////////////////////////////////////////
template <typename S, typename Tag,
          typename CoarseTag, typename ExecutionTag>
struct transform<S, tags::scan_reduce<Tag, tags::inclusive>,
                 tags::coarse<CoarseTag, ExecutionTag>>
{
  using value_t = typename S::value_t;

  static auto call(S const& skeleton)
    STAPL_AUTO_RETURN((compose<skeletons::tags::inline_flow>(
      ph::x<0>() << map(
        skeletons::wrap<ExecutionTag>(skeletons::reduce(skeleton.get_op()))) |
        ph::input<0>(),
      ph::x<1>() << skeletons::scan<Tag>(skeleton.get_op()) | ph::x<0>(),
      ph::x<2>() << skeletons::elem(
        skeletons::scan_helpers::find_scan_update_phase_pd::
          call<tags::inclusive, CoarseTag>(skeleton.get_op())) |
        (ph::x<1>(), ph::input<0>(), ph::input<1>()),
      ph::x<3>() << skeletons::pre_broadcast<1>(
        stapl::identity<value_t>(),
        skeleton_traits<spans::only_nearest_pow_two>()) |
        ph::x<1>(),
      ph::x<4>() << skeletons::broadcast_to_locs<true, tags::right_aligned>() |
        ph::x<3>())))
};


} // namespace transformations
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_COARSE_SCAN_REDUCE_HPP

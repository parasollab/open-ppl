/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_COARSE_COMPOSE_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_COARSE_COMPOSE_HPP

#include <stapl/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/transformations/transform.hpp>

#include <stapl/skeletons/transformations/coarse/coarse.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/utility/integer_sequence.hpp>


namespace stapl {
namespace skeletons {
namespace transformations {

template <typename S, typename SkeletonTag, typename CoarseTag>
struct transform;


//////////////////////////////////////////////////////////////////////
/// @brief A coarsening transformation on a composed skeleton
/// applies coarsening on each individual skeleton and composes them
/// again using the flow specification of the original fine-grain
/// composition.
///
/// @tparam S            the composed skeleton to be coarsened
/// @tparam Flows        the flows specification used in the original
///                      composition
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
template<typename... S, typename Flows,
         typename SkeletonTag, typename CoarseTag, typename ExecutionTag>
struct transform<stapl::skeletons::skeletons_impl::compose<
                   stapl::tuple<S...>, Flows>,
                 SkeletonTag,
                 tags::coarse<CoarseTag, ExecutionTag>>
{
private:
  using skeleton_t = stapl::skeletons::skeletons_impl::compose<
                       stapl::tuple<S...>, Flows>;
  using flows_t    = Flows;
  using ct         = CoarseTag;
  using et         = ExecutionTag;

  template <std::size_t... Indices>
  static auto
  apply_coarse(skeleton_t const& c, stapl::index_sequence<Indices...>&&)
  STAPL_AUTO_RETURN((
    stapl::skeletons::compose<flows_t>(
      stapl::skeletons::coarse<ct, et>(
        c.template get_skeleton<Indices>())...)
  ))

public:
  static auto call(skeleton_t const& skeleton)
  STAPL_AUTO_RETURN((
    apply_coarse(skeleton, stapl::make_index_sequence<sizeof...(S)>())
  ))
};


template<typename Skeleton, typename S, typename CoarseTag,
         typename ExecutionTag>
struct transform<Skeleton,
                 tags::sink_value<S>,
                 tags::coarse<CoarseTag, ExecutionTag>>
  : public transform<typename Skeleton::base_type,
                     typename Skeleton::base_type::skeleton_tag_type,
                     tags::coarse<CoarseTag, ExecutionTag>>
{ };


} // namespace transformations
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_COARSE_COMPOSE_HPP

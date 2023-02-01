/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_NEST_WAVEFRONT_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_NEST_WAVEFRONT_HPP


#include <stapl/skeletons/transformations/transform.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/transformations/optimizers/nested.hpp>
#include <stapl/skeletons/functional/wavefront.hpp>
#include <stapl/skeletons/utility/mapper_utils.hpp>

namespace stapl {
namespace skeletons {
namespace transformations {

//////////////////////////////////////////////////////////////////////
/// @brief Adds the appropriate mappers and filters needed for the
///        nested execution of wavefront skeleton. When the operator
///        of the skeleton is another skeleton, the execution params
///        of the operator need to be preserved during the transformation.
///
/// @ingroup skeletonsTransformationsNest
//////////////////////////////////////////////////////////////////////
template <typename S,
          typename Tag,
          bool isNested = is_nested_skeleton<typename S::op_type>::value>
struct transform_impl
{
  constexpr static size_t dims = S::span_type::nested_dims_num::value;
  using mappers_t = mappers<dims, Tag>;

  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::wavefront<S::number_of_inputs>(
      nest(get_op(skeleton.get_op())),
      skeleton.get_start_corner(),
      skeletons::skeleton_traits<typename S::span_type, S::set_result>(
        get_nested_filter<dims>(skeleton.get_filter(), skeleton.get_op()),
        mappers_t(skeleton.get_start_corner())),
      skeleton.get_op().get_exec_params()
    )
  ))
};

//////////////////////////////////////////////////////////////////////
/// @brief Adds the appropriate mappers and filters needed for the
///        nested execution of wavefront skeleton. Specialization when
///        the operator of skeleton is not another skeleton, so there is
///        no execution params to be preserved during the transformation.
///
/// @ingroup skeletonsTransformationsNest
//////////////////////////////////////////////////////////////////////
template <typename S, typename Tag>
struct transform_impl<S, Tag, false>
{
  constexpr static size_t dims = S::span_type::nested_dims_num::value;
  using mappers_t = mappers<dims, Tag>;

  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::wavefront<S::number_of_inputs>(
      nest(get_op(skeleton.get_op())),
      skeleton.get_start_corner(),
      skeletons::skeleton_traits<typename S::span_type, S::set_result>(
        get_nested_filter<dims>(skeleton.get_filter(), skeleton.get_op()),
        mappers_t(skeleton.get_start_corner()))
    )
  ))
};


template <typename S, typename SkeletonTag, typename CoarseTag>
struct transform;

//////////////////////////////////////////////////////////////////////
/// @brief Adds the appropriate mappers and filters needed for the
///        nested execution of wavefront skeleton.
///
/// @ingroup skeletonsTransformationsNest
//////////////////////////////////////////////////////////////////////
template <typename S, int Dim>
struct transform<S, tags::wavefront<Dim>, tags::recursive_nest>
{
  using skeleton_tag_t = tags::wavefront<Dim>;

  constexpr static size_t dims = S::span_type::nested_dims_num::value;
  using mappers_t = mappers<dims, tags::wavefront<Dim>>;

  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    transform_impl<S, skeleton_tag_t>::call(skeleton)
  ))
};


} // namespace transformations
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_NEST_WAVEFRONT_HPP

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_NEST_ZIP_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_NEST_ZIP_HPP


#include <stapl/skeletons/transformations/transform.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/transformations/optimizers/nested.hpp>
#include <stapl/skeletons/functional/zip.hpp>

namespace stapl {
namespace skeletons {
namespace transformations {

template <typename S, typename SkeletonTag, typename TransformTag>
struct transform;



//////////////////////////////////////////////////////////////////////
/// @brief Adds the appropriate mappers and filters needed for the
///        nested execution of zip skeleton.
///
/// @ingroup skeletonsTransformationsNest
//////////////////////////////////////////////////////////////////////
template <typename S, typename Tag, int Arity>
struct transform<S, tags::zip<Tag, Arity>, tags::recursive_nest>
{

  using skeleton_tag_t = tags::zip<Tag, Arity>;
  using Dim            = typename S::span_type::nested_dims_num;
  using mappers_t      = mappers<Dim::value, skeleton_tag_t>;

  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::zip<Arity>(
      nest(get_op(skeleton.get_op())),
      skeleton_traits<typename S::span_type, S::set_result>(
        get_nested_filter<Dim::value>(skeleton.get_filter(), skeleton.get_op()),
        mappers_t())
    )
  ))
};

} // namespace transformations
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_NEST_ZIP_HPP

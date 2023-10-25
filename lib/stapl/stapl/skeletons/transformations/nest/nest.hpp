/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_TRANSFORMATIONS_NEST_NEST_HPP
#define STAPL_SKELETONS_TRANSFORMATIONS_NEST_NEST_HPP

#include <stapl/skeletons/transformations/transform.hpp>
#include <stapl/skeletons/transformations/wrapped_skeleton.hpp>
#include <stapl/skeletons/transformations/optimizers/nested.hpp>

#include <stapl/skeletons/functional/map.hpp>
#include <stapl/skeletons/functional/reduce_to_locs.hpp>
#include <stapl/skeletons/functional/broadcast_to_locs.hpp>

namespace stapl {
namespace skeletons {
namespace transformations {

template <typename S, typename SkeletonTag, typename TransformTag>
struct transform;


//////////////////////////////////////////////////////////////////////
/// @brief The transformation for a @ref skeletons::reduce in a nested
/// section (e.g., map(reduce)) transforms it to a
/// @ref skeletons::reduce_to_locs. In addition, for performance reasons
/// it should also add a map(identity<T>) to avoid remote fetches
/// from a view.
///
/// @see make_paragraph_skeleton_manager
/// @todo The map(identity<T>) can be removed whenever fetches from
/// views are changed to future-then.
///
/// @ingroup skeletonsTransformationsNest
//////////////////////////////////////////////////////////////////////
template <typename S, typename Tag>
struct transform<S, tags::reduce<Tag>, tags::nest>
{
private:
  using value_t = typename S::op_type::result_type;

public:
  template <typename ExecutionParams>
  static auto call(S const& skeleton, ExecutionParams const& execution_params)
  STAPL_AUTO_RETURN((
    skeletons::wrap<tags::nested_execution<true>>(
      skeletons::compose(
        skeletons::map(stapl::identity<value_t>()),
        skeletons::reduce_to_locs<true>(skeleton.get_op())),
        execution_params)
  ))

  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::wrap<tags::nested_execution<true>>(
      skeletons::compose(
        skeletons::map(stapl::identity<value_t>()),
        skeletons::reduce_to_locs<true>(skeleton.get_op())))
  ))
};


//////////////////////////////////////////////////////////////////////
/// @brief The transformation for a @ref skeletons::zip_reduce in a nested
/// section (e.g., zip(reduce)) composes it with a @c broadcast_to_locs
/// skeleton.
///
/// @see make_paragraph_skeleton_manager
///
/// @ingroup skeletonsTransformationsNest
//////////////////////////////////////////////////////////////////////
template <typename S, int i>
struct transform<S, tags::zip_reduce<i>, tags::nest>
{
private:
  using value_t   = typename S::reduce_op_type::result_type;

public:
  template <typename ExecutionParams>
  static auto call(S const& skeleton, ExecutionParams const& execution_params)
  STAPL_AUTO_RETURN((
    skeletons::wrap<tags::nested_execution<true>>(
      skeletons::compose(skeleton, skeletons::broadcast_to_locs<true>()),
      execution_params)
  ))

  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::wrap<tags::nested_execution<true>>(
      skeletons::compose(skeleton, skeletons::broadcast_to_locs<true>()))
  ))
};


//////////////////////////////////////////////////////////////////////
/// @brief In the default case, a transformation of a skeleton with
/// the @c tags::nest tag should not modify the skeleton. For example,
/// a nested @ref skeletons::map or @ref skeletons::zip can be executed
/// without applying any transformations.
///
/// @see make_paragraph_skeleton_manager
///
/// @ingroup skeletonsTransformationsNest
//////////////////////////////////////////////////////////////////////
template <typename S, typename Tag>
struct transform<S, Tag, tags::nest>
{
private:

public:
  template <typename ExecutionParams>
  static auto call(S const& skeleton, ExecutionParams const& execution_params)
  STAPL_AUTO_RETURN((
    skeletons::wrap<tags::nested_execution<false>>(
      skeleton, execution_params)
  ))

  static auto call(S const& skeleton)
  STAPL_AUTO_RETURN((
    skeletons::wrap<tags::nested_execution<false>>(
      skeleton)
  ))
};

} // namespace transformations


//////////////////////////////////////////////////////////////////////
/// @brief no transformation for non skeleton operators
///
/// @param skeleton     the skeleton to be nested
///
/// @ingroup skeletonsTransformations
//////////////////////////////////////////////////////////////////////
template <typename Op,
          typename =
            typename std::enable_if<
              !is_skeleton<typename std::decay<Op>::type>::value>::type>
Op nest(Op&& op)
{
  return op;
}


//////////////////////////////////////////////////////////////////////
/// @brief recursive calls to the nested skeletons for setting their
///        proper filters and mappers for paragraph ports
///
/// @param skeleton     the skeleton to be nested
///
/// @ingroup skeletonsTransformations
//////////////////////////////////////////////////////////////////////
template <typename S,
          typename =
            typename std::enable_if<
              is_skeleton<typename std::decay<S>::type>::value>::type>
auto
nest(S&& skeleton)
STAPL_AUTO_RETURN((
  skeletons::transform<tags::recursive_nest>(
    std::forward<S>(skeleton))
))


} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_TRANSFORMATIONS_NEST_NEST_HPP

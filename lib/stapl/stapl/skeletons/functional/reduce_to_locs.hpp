/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_REDUCE_TO_LOCS_HPP
#define STAPL_SKELETONS_FUNCTIONAL_REDUCE_TO_LOCS_HPP

#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include "reduce.hpp"
#include "broadcast_to_locs.hpp"

namespace stapl {
namespace skeletons {

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a reduce to location
/// skeleton by exposing only the necessary information in its
/// representation.
///
/// A reduce_to_locs skeleton first reduces the input and then
/// broadcasts the result to all locations using a @c broadcast_to_locs
/// skeleton.
///
/// This abstraction not only makes the reconstruction of a
/// reduce_to_locs skeleton easier, but also provides access to the
/// underlying operation in the reduce skeleton. Furthermore, it reduces
/// the symbol size for a reduce_to_locs skeleton, hence, reducing the
/// total compilation time.
///
/// @tparam Op        the operation to be used while reducing the input.
/// @tparam Flows     the flow to be used between the reduction and
///                   the broadcast_to_locs skeletons.
/// @tparam Span      the iteration space for the elements in each level
///                   of the reduction skeleton.
/// @tparam Tag       determines the type of reduction and broadcast to be
///                   used.
/// @tparam SetResult whether the skeleton should set the task
///                   results on the pg edge container or not
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
namespace skeletons_impl {
template <typename Op,
          typename Flows,
          typename Span,
          typename Tag,
          bool SetResult>
struct reduce_to_locs
  : public decltype(
             skeletons::compose<Flows>(
               skeletons::reduce<Tag, stapl::use_default, Span>(
                 std::declval<Op>()),
               skeletons::broadcast_to_locs<SetResult, Tag>()))
{
  using skeleton_tag_type = tags::reduce_to_locs;
  using op_type = Op;

  using base_type = decltype(
                      skeletons::compose<Flows>(
                        skeletons::reduce<Tag, stapl::use_default, Span>(
                          std::declval<Op>()),
                        skeletons::broadcast_to_locs<SetResult, Tag>()));

  reduce_to_locs(Op const& op)
    : base_type(
        skeletons::compose<Flows>(
          skeletons::reduce<Tag, stapl::use_default, Span>(op),
          skeletons::broadcast_to_locs<SetResult, Tag>())
      )
  { }

  Op get_op(void) const
  {
    return base_type::template get_skeleton<0>().
             nested_skeleton().nested_skeleton().get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

}

namespace result_of {

template <typename Tag,
          typename Flows,
          typename Span,
          typename Op,
          bool SetResult>
using reduce_to_locs  =
  skeletons_impl::reduce_to_locs<
    typename std::decay<Op>::type, Flows, Span, Tag, SetResult>;

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief A reduce to location skeleton is used to obtain the reduced
/// value of an input/computation on every locations.
///
/// This skeleton is obtained by putting a @ref reduce skeleton and a
/// @ref broadcast_to_loc skeleton together. This skeleton can be used
/// with all left-aligned reduce and broadcast skeletons.
///
/// @tparam SetResult whether the skeleton should set the task
///                   results on the pg edge container or not
/// @tparam Tag       determines the type of span and reduce parametric
///                   dependency to be used
/// @tparam Flows     the flows to be used in the reduction skeleton iterations
/// @tparam Span      the iteration space for the elements in the
///                   reduction tree
/// @param  op        the operation (an element-wise binary functor) to be
///                   used in each reduce parametric dependency
/// @return a reduce skeleton with a tag-determined parametric dependency
///
/// @see tree
///
/// @ingroup skeletonsFunctionalReduce
//////////////////////////////////////////////////////////////////////
template <bool SetResult  = false,
          typename Tag    = stapl::use_default,
          typename Flows  = stapl::use_default,
          typename Span   = stapl::use_default,
          typename Op>
result_of::reduce_to_locs<Tag, Flows, Span, Op, SetResult>
reduce_to_locs(Op&& op)
{
  return result_of::reduce_to_locs<Tag, Flows, Span, Op, SetResult>(
           std::forward<Op>(op));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_REDUCE_TO_LOCS_HPP

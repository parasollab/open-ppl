/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_BROADCAST_HPP
#define STAPL_SKELETONS_FUNCTIONAL_BROADCAST_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/spans/balanced.hpp>
#include "reverse_binary_tree.hpp"

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a broadcast skeleton
/// by exposing only the necessary information in its representation.
///
/// A broadcast skeleton is simply a broadcast tree, however, since
/// a broadcast tree can only handle an input of power-of-two sizes,
/// this skeleton composes a broadcast tree to a an expansion skeleton.
///
/// This abstraction not only makes the reconstruction of a broadcast
/// skeleton easier, but also provides access to the underlying
/// operation of a broadcast skeleton. Furthermore, it reduces the
/// symbol size for a broadcast skeleton, hence, reducing the
/// total compilation time.
///
/// @tparam Op    the operation to be used while broadcasting.
/// @tparam Flows the flow to be used for the @c broadcast_tree.
/// @tparam Span  the iteration space for elements on each level of
///               the broadcast tree
/// @tparam  tag  determines the type of the broadcast skeleton
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Flows, typename Span,
          typename Tag, bool SetResult>
struct broadcast
  : public decltype(
             skeletons::reverse_binary_tree<SetResult, Tag, Flows, Span>(
               std::declval<Op>())
           )
{
  using skeleton_tag_type = tags::broadcast<Tag>;
  using base_type = decltype(
                      skeletons::reverse_binary_tree<
                        SetResult, Tag, Flows, Span>(
                          std::declval<Op>()));

  explicit broadcast(Op const& op)
    : base_type(skeletons::reverse_binary_tree<
                  SetResult, Tag, Flows, Span>(op))
  { }

  Op get_op(void) const
  {
    return base_type::get_op();
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
using broadcast = skeletons_impl::broadcast<
                    typename std::decay<Op>::type,
                    Flows,
                    stapl::default_type<Span, spans::balanced<>>,
                    stapl::default_type<Tag, tags::left_aligned>,
                    SetResult
                  >;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief A broadcast tree is a common skeleton that is used in
/// various applications, e.g., allreduce, reduce_to_loc, etc. A
/// broadcast skeleton in the skeletons library is a @c reverse_tree
/// made out of a broadcast parametric dependency. The behavior of the
/// enclosed broadcast parametric dependency changes based on the
/// given @c tag (@see reverse_binary_tree_pd)
///
/// If tag tag is :
/// @li @c tags::left_aligned - creates the default broadcast skeleton
///     which is used in various skeletons.
/// @li @c tags::right_aligned - creates a right-aligned broadcast
///     tree is which used in skeletons such as @c scan_blelloch
/// @li @c stapl::left_skewed - the most unbalanced broadcast skeleton
///
/// @tparam Flows     the flow to be used for the @c reverse_tree. Some
///                   skeletons need a flow other than the default one
///                   for a tree
/// @tparam Span      the iteration space for elements on each level of
///                   the tree
/// @param  op        the operation (an element-wise unary functor) to be
///                   used in each broadcast parametric dependency. Usually,
///                   @c stapl::identity is used
/// @tparam Tag       determines the type of the broadcast skeleton
/// @return a broadcast skeleton
///
/// @tparam SetResult whether the skeleton should set the task
///                   results on the pg edge container or not
///
/// @see tags::right_aligned
/// @see tags::left_aligned
///
/// @ingroup skeletonsFunctionalBroadcast
//////////////////////////////////////////////////////////////////////
template <typename Tag    = stapl::use_default,
          typename Flows  = stapl::use_default,
          typename Span   = stapl::use_default,
          bool SetResult  = false,
          typename Op>
result_of::broadcast<Tag, Flows, Span, Op, SetResult>
broadcast(Op&& op)
{
  return result_of::broadcast<
           Tag, Flows, Span, Op, SetResult>(std::forward<Op>(op));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_BROADCAST_HPP

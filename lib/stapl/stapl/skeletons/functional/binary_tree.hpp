/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_BINARY_TREE_HPP
#define STAPL_SKELETONS_FUNCTIONAL_BINARY_TREE_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/spans/balanced.hpp>
#include <stapl/skeletons/spans/nearest_pow_two.hpp>
#include <stapl/skeletons/spans/tree.hpp>
#include <stapl/skeletons/param_deps/binary_tree_pd.hpp>
#include "reduce_to_pow_two.hpp"
#include "tree.hpp"

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

template <typename Op, typename Flows, typename Span, typename Tag,
          bool isComplete>
struct binary_tree;

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a complete binary tree
/// by exposing only the necessary information in its representation.
///
/// This abstraction not only makes the reconstruction of a
/// binary_tree skeleton easier, but also provides access to the
/// underlying reduction operation. Furthermore, it reduces the symbol
/// size for a binary_tree skeleton, hence, reducing the total
/// compilation time.
///
/// @tparam Op    the operation to be used for reduction.
/// @tparam Flows the flow between the levels of the binary tree.
/// @tparam Span  the iteration space for the elements in each level
///               of each level.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Flows, typename Span, typename Tag>
struct binary_tree<Op, Flows, Span, Tag, true>
  : public decltype(
             skeletons::tree<2, Flows, spans::tree<Span, Tag>>(
               skeletons::binary_tree_pd<Tag>(std::declval<Op>()))
           )
{
  using skeleton_tag_type = tags::binary_tree<Tag>;
  using op_type = Op;
  using base_type = decltype(
                      skeletons::tree<2, Flows, spans::tree<Span, Tag>>(
                        skeletons::binary_tree_pd<Tag>(std::declval<Op>())));


  explicit binary_tree(Op const& op)
    : base_type(
        skeletons::tree<2, Flows, spans::tree<Span, Tag>>(
          skeletons::binary_tree_pd<Tag>(op))
      )
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

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a non-complete binary
/// tree by exposing only the necessary information in its representation.
///
/// This skeleton is used in several composed skeletons such as a
/// @c reduce skeleton.
///
/// This abstraction not only makes the reconstruction of a
/// binary_tree skeleton easier, but also provides access to the
/// underlying reduction operation. Furthermore, it reduces the symbol
/// size for a binary_tree skeleton, hence, reducing the total
/// compilation time.
///
/// @tparam Op    the operation to be used for reduction.
/// @tparam Flows the flow between the levels of the binary tree.
/// @tparam Span  the iteration space for the elements in each level
///               of each level.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Flows, typename Span, typename Tag>
struct binary_tree<Op, Flows, Span, Tag, false>
  : public decltype(
             skeletons::compose(
               skeletons::reduce_to_pow_two<Span>(std::declval<Op>()),
               skeletons_impl::binary_tree<
                 Op, Flows, spans::nearest_pow_two<Span>, Tag, true
               >(std::declval<Op>())
             )
           )
{
  using op_type = Op;
  using skeleton_tag_type = tags::binary_tree<Tag>;
  using base_type = decltype(
                      skeletons::compose(
                        skeletons::reduce_to_pow_two<Span>(std::declval<Op>()),
                        skeletons_impl::binary_tree<
                          Op, Flows, spans::nearest_pow_two<Span>, Tag, true
                        >(std::declval<Op>())));


  explicit binary_tree(Op const& op)
    : base_type(
        skeletons::compose(
          skeletons::reduce_to_pow_two<Span>(op),
          skeletons_impl::binary_tree<
            Op, Flows, spans::nearest_pow_two<Span>, Tag, true
          >(op)
        )
      )
  { }

  Op get_op(void) const
  {
    return base_type::template get_skeleton<1>().get_op();
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
          bool isComplete,
          typename Op>
using binary_tree = skeletons_impl::binary_tree<
                      typename std::decay<Op>::type,
                      Flows,
                      stapl::default_type<Span, spans::balanced<>>,
                      stapl::default_type<Tag, tags::left_aligned>,
                      isComplete>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief A binary tree used in many algorithms including but not
/// limited to reduction, and prefix scan.
///
/// A binary tree can be created in various ways based on the
/// tag provided to it:
/// @li tags::left_aligned a left aligned binary tree with a left
///     aligned @c binary_tree_pd
/// @li tags::right_aligned a right aligned binary tree with a right
///     aligned @c binary_tree_pd
/// @li stapl::use_default a default binary tree with default
///     @c binary_tree_pd is used.
///
/// @tparam Span  the iteration space for the elements in the
///               reduction tree
/// @tparam Flows the flow to be used for this skeleton
/// @param  op    the operation (an element-wise binary functor) to be
///               used in each @c binary_tree_pd
/// @param  tag   determines the type of span and @c binary_tree_pd
///               to be used
/// @return a binary tree skeleton with a tag-determined parametric
///         dependency
///
/// @see tree
///
/// @ingroup skeletonsFunctionalReduce
//////////////////////////////////////////////////////////////////////
template <typename Tag    = stapl::use_default,
          typename Flows  = stapl::use_default,
          typename Span   = stapl::use_default,
          bool isComplete = false,
          typename Op>
result_of::binary_tree<Tag, Flows, Span, isComplete, Op>
binary_tree(Op&& op)
{
  return result_of::binary_tree<Tag, Flows, Span, isComplete, Op>(
           std::forward<Op>(op));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_BINARY_TREE_HPP

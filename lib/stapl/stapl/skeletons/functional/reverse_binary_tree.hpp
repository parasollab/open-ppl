/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_REVERSE_BINARY_TREE_HPP
#define STAPL_SKELETONS_FUNCTIONAL_REVERSE_BINARY_TREE_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/spans/balanced.hpp>
#include <stapl/skeletons/spans/nearest_pow_two.hpp>
#include <stapl/skeletons/spans/tree.hpp>
#include <stapl/skeletons/param_deps/reverse_binary_tree_pd.hpp>
#include "expand_from_pow_two.hpp"
#include "tree.hpp"
#include "../operators/compose.hpp"

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

template <typename Op, typename Flows, typename Span, typename Tag,
          bool isComplete, typename Filter, bool isPositionAware,
          bool SetResult>
struct reverse_binary_tree;

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a reverse binary tree
/// by exposing only the necessary information in its representation.
///
/// This skeleton is used in several composed skeletons such as a
/// @c broadcast skeleton.
///
/// This abstraction not only makes the reconstruction of a
/// reverse_binary_tree skeleton easier, but also provides access to the
/// underlying reduction operation. Furthermore, it reduces the symbol
/// size for a reverse_binary_tree skeleton, hence, reducing the total
/// compilation time.
///
/// @tparam Op    the operation to be used for reduction.
/// @tparam Flows the flow between the levels of the binary tree.
/// @tparam Span  the iteration space for the elements in each level
///               of each level.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Flows, typename Span, typename Tag,
          typename Filter, bool isPositionAware, bool SetResult>
struct reverse_binary_tree<Op, Flows, Span, Tag,
                           true, Filter, isPositionAware, SetResult>
  : public decltype(
             skeletons::reverse_tree<2, Flows, spans::reverse_tree<Span, Tag>>(
               skeletons::reverse_binary_tree_pd<
                 Tag, isPositionAware, SetResult>(
                   std::declval<Op>(), std::declval<Filter>())))
{
  using skeleton_tag_type = tags::reverse_binary_tree<Tag>;
  using op_type = Op;

  using base_type = decltype(
                      skeletons::reverse_tree<
                        2, Flows, spans::reverse_tree<Span, Tag>
                      >(skeletons::reverse_binary_tree_pd<
                          Tag, isPositionAware, SetResult>(
                            std::declval<Op>(), std::declval<Filter>())));

  reverse_binary_tree(Op const& op, Filter const& filter)
    : base_type(
        skeletons::reverse_tree<2, Flows, spans::reverse_tree<Span, Tag>>(
          skeletons::reverse_binary_tree_pd<Tag, isPositionAware, SetResult>
            (op, filter))
      )
  { }

  Op get_op(void) const
  {
    return base_type::get_op();
  }

  Filter get_filter(void) const
  {
    return base_type::get_filter();
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
/// @c broadcast skeleton.
///
/// This abstraction not only makes the reconstruction of a
/// reverse_binary_tree skeleton easier, but also provides access to the
/// underlying reduction operation. Furthermore, it reduces the symbol
/// size for a reverse_binary_tree skeleton, hence, reducing the total
/// compilation time.
///
/// @tparam Op    the operation to be used for reduction.
/// @tparam Flows the flow between the levels of the binary tree.
/// @tparam Span  the iteration space for the elements in each level
///               of each level.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Flows, typename Span, typename Tag,
          typename Filter, bool isPositionAware, bool SetResult>
struct reverse_binary_tree<Op, Flows, Span, Tag, false,
                           Filter, isPositionAware, SetResult>
  : public decltype(
             skeletons::compose(
               skeletons_impl::reverse_binary_tree<
                 Op, Flows, spans::nearest_pow_two<Span>, Tag, true, Filter,
                 isPositionAware, false
               >(std::declval<Op>(), std::declval<Filter>()),
               skeletons::expand_from_pow_two<
                 Span, isPositionAware, SetResult>(
                   std::declval<Op>(), std::declval<Filter>())
             )
           )
{
  using skeleton_tag_type = tags::reverse_binary_tree<Tag>;
  using op_type = Op;

  using base_type = decltype(
                      skeletons::compose(
                        skeletons_impl::reverse_binary_tree<
                          Op, Flows, spans::nearest_pow_two<Span>, Tag,
                          true, Filter, isPositionAware, false
                        >(std::declval<Op>(), std::declval<Filter>()),
                        skeletons::expand_from_pow_two<
                          Span, isPositionAware, SetResult>(
                            std::declval<Op>(), std::declval<Filter>())));

  reverse_binary_tree(Op const& op, Filter const& filter)
    : base_type(
        skeletons::compose(
          skeletons_impl::reverse_binary_tree<
            Op, Flows, spans::nearest_pow_two<Span>, Tag, true, Filter,
            isPositionAware, false
          >(op, filter),
          skeletons::expand_from_pow_two<Span, isPositionAware, SetResult>
            (op, filter)))
  { }

  Op get_op(void) const
  {
    return base_type::template get_skeleton<1>().get_op();
  }

  Filter get_filter(void) const
  {
    return base_type::template get_skeleton<1>().get_filter();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

}

namespace result_of {

template <typename Tag, typename Flows, typename Span,
          bool isComplete, bool isPositionAware,
          typename Op, typename Filter, bool SetResult>
using reverse_binary_tree = skeletons_impl::reverse_binary_tree<
                              typename std::decay<Op>::type, Flows,
                              stapl::default_type<Span, spans::balanced<>>,
                              stapl::default_type<Tag, tags::left_aligned>,
                              isComplete,
                              typename std::decay<Filter>::type,
                              isPositionAware,
                              SetResult>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief A reverse binary tree used in many other skeletons such as
/// broadcast.
///
/// A reverse binary tree can be created in various ways based on the
/// tag provided to it:
/// @li tags::left_aligned a left aligned reverse binary tree with a left
///     aligned @c reverse_binary_tree_pd
/// @li tags::right_aligned a right aligned reverse binary tree with a right
///     aligned @c reverse_binary_tree_pd
/// @li stapl::use_default a default reverse binary tree with default
///     @c reverse_binary_tree_pd is used.
///
/// @tparam SetResult       whether the skeleton should set the task
///                         results on the pg edge container or not
/// @tparam Tag             determines the type of span and
///                         @c reverse_binary_tree_pd
///                         to be used
///                         reduction tree
/// @tparam Flows           the flow to be used for this skeleton
/// @tparam Span            the iteration space for the elements in the
/// @tparam isPositionAware whether the op needs to know the position of
///                         the node in the dependence graph or not
/// @param  op              the operation (an element-wise binary functor)
///                         to be used in each @c reverse_binary_tree_pd
///
/// @return a reverse binary tree skeleton with a tag-determined parametric
///         dependency
///
/// @see tree
///
/// @ingroup skeletonsFunctionalReduce
//////////////////////////////////////////////////////////////////////
template <bool SetResult        = false,
          typename Tag          = stapl::use_default,
          typename Flows        = stapl::use_default,
          typename Span         = stapl::use_default,
          bool isComplete       = false,
          bool isPositionAware  = false,
          typename Op,
          typename Filter       = skeletons::no_filter>
result_of::reverse_binary_tree<
  Tag, Flows, Span, isComplete,isPositionAware, Op, Filter, SetResult>
reverse_binary_tree(Op&& op, Filter&& filter = Filter())
{
  return result_of::reverse_binary_tree<
           Tag, Flows, Span, isComplete, isPositionAware, Op, Filter, SetResult
         >(std::forward<Op>(op), std::forward<Filter>(filter));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_REVERSE_BINARY_TREE_HPP

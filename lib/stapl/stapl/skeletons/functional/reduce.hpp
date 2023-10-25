/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_REDUCE_HPP
#define STAPL_SKELETONS_FUNCTIONAL_REDUCE_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include "binary_tree.hpp"

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a reduce skeleton
/// by exposing only the necessary information in its representation.
///
/// A reduce skeleton consists of a reducing the input to the closest
/// power-of-two and then performing a reduction using a binary
/// reduction tree on that. The reduction tree can have several
/// shapes and one can modify its layout by specifying the @c Tag
/// parameter.
///
/// This abstraction not only makes the reconstruction of a reduce
/// skeleton easier, but also provides access to the underlying operation
/// in the reduce skeleton. Furthermore, it reduces the symbol size
/// for a reduce skeleton, hence, reducing the total compilation time.
///
/// @tparam Op    the operation to be used while reducing the input.
/// @tparam Flows the flow to be used between the levels of the
///               reduction tree.
/// @tparam Span  the iteration space for the elements in each level
///               of both the reduction to power-of-two and the
///               reduction tree.
/// @tparam Tag   determines the type of reduction to be used.
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Flows, typename Span, typename Tag>
struct reduce
  : public decltype(
             skeletons::binary_tree<Tag, Flows, Span>(std::declval<Op>()))
{
  using skeleton_tag_type = tags::reduce<Tag>;
  using op_type = Op;
  using base_type = decltype(
                      skeletons::binary_tree<Tag, Flows, Span>(
                        std::declval<Op>()));

  explicit reduce(Op const& op)
    : base_type(skeletons::binary_tree<Tag, Flows, Span>(op))
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

} // namespace skeletons_impl


namespace result_of {

template <typename Tag,
          typename Flows,
          typename Span,
          typename Op>
using reduce = skeletons_impl::reduce<
                 typename std::decay<Op>::type, Flows,
                 stapl::default_type<Span, spans::balanced<>>,
                 stapl::default_type<Tag, tags::left_aligned>>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief A reduce skeleton is used in many algorithms, e.g., reduction,
/// allreduce, scan, etc. This skeleton is a tree of reduce parametric
/// dependency. The type of the tree, span, and the parametric dependency
/// to be used in this skeleton is determined by the given @c tag. A few
/// examples of reduce skeleton variations are when the following
/// tags are used:
/// @li tags::left_aligned a left aligned reduce tree with a left
///     aligned reduce parametric dependency
/// @li tags::right_aligned a right aligned reduce tree with a right
///     aligned reduce parametric dependency
/// @li stapl::use_default a default reduce tree with default reduce
///     parametric dependency is used.
///
/// @note This reduction implementation assumes that the reduction
///       operation is both associative and commutative.
///
/// @tparam Span  the iteration space for the elements in the
///               reduction tree
/// @tparam Flows the flow to be used for this skeleton
/// @param  op    the operation (an element-wise binary functor) to be
///               used in each reduce parametric dependency
/// @param  tag   determines the type of span and reduce parametric
///               dependency to be used
/// @return a reduce skeleton with a tag-determined parametric dependency
///
/// @see tree
///
/// @ingroup skeletonsFunctionalReduce
//////////////////////////////////////////////////////////////////////
template <typename Tag   = stapl::use_default,
          typename Flows = stapl::use_default,
          typename Span  = stapl::use_default,
          typename Op>
result_of::reduce<Tag, Flows, Span, Op>
reduce(Op&& op)
{
  return result_of::reduce<Tag, Flows, Span, Op>(std::forward<Op>(op));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_REDUCE_HPP

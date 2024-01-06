/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_REDUCE_TO_POW_TWO_HPP
#define STAPL_SKELETONS_FUNCTIONAL_REDUCE_TO_POW_TWO_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/param_deps/reduce_to_pow_two_pd.hpp>
#include <stapl/skeletons/spans/balanced.hpp>
#include <stapl/skeletons/spans/reduce_to_pow_two.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a reduce to
/// power-of-two skeleton by exposing only the necessary information in
/// its representation.
///
/// A reduce_to_pow_two skeleton converts the input to its closest
/// power-of-two size by combining input elements using the operation
/// provided.
///
/// This abstraction not only makes the reconstruction of a
/// reduce_to_pow_two skeleton easier, but also provides access to the
/// underlying operation in the reduce skeleton. Furthermore, it reduces
/// the symbol size for a reduce_to_pow_two skeleton, hence, reducing the
/// total compilation time.
///
/// @tparam Op    the operation to be used while reducing the input.
/// @tparam Span  the iteration space for the elements.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Span>
struct reduce_to_pow_two
  : public decltype(
             skeletons::elem<Span>(
               skeletons::reduce_to_pow_two_pd(std::declval<Op>())))
{
  using skeleton_tag_type = tags::reduce_to_pow_two;
  using base_type = decltype(
                      skeletons::elem<Span>(
                        skeletons::reduce_to_pow_two_pd(std::declval<Op>())));

  explicit reduce_to_pow_two(Op const& op)
    : base_type(skeletons::elem<Span>(skeletons::reduce_to_pow_two_pd(op)))
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

template <typename Span, typename Op>
using reduce_to_pow_two = skeletons_impl::reduce_to_pow_two<
                            typename std::decay<Op>::type,
                            spans::reduce_to_pow_two<
                              stapl::default_type<Span, spans::balanced<>>>>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief This skeleton is used as the pre-processing phase of various
/// skeletons including reduce, n-partition, etc. Basically, this
/// skeleton tries to prepare the input for a pow-two based skeleton.
///
/// @param  op    the operation to be used in order to reduce the
///               input size to the closest pow-of-two size
/// @tparam Span  the inner span on which the @c reduce_to_pow_two
///               should be defined
/// @return a reduce to power two skeleton with given span and operation
///
/// @see reduce
///
/// @ingroup skeletonsFunctionalReduce
//////////////////////////////////////////////////////////////////////
template <typename Span = stapl::use_default,
          typename Op>
result_of::reduce_to_pow_two<Span, Op>
reduce_to_pow_two(Op&& op)
{
  return result_of::reduce_to_pow_two<Span, Op>(std::forward<Op>(op));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_REDUCE_TO_POW_TWO_HPP

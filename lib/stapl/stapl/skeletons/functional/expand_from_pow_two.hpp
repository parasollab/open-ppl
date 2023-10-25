/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_EXPAND_FROM_POW_TWO_HPP
#define STAPL_SKELETONS_FUNCTIONAL_EXPAND_FROM_POW_TWO_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/param_deps/expand_from_pow_two_pd.hpp>
#include <stapl/skeletons/spans/balanced.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of an expansion skeleton
/// by exposing only the necessary information in its representation.
///
/// An expansion skeleton is usually used along with a skeleton (S) that
/// only handle inputs of power-of-two sizes and converts the
/// produced output by skeleton to arbitrary sizes.
///
/// This abstraction not only makes the reconstruction of an
/// expansion skeleton easier, but also provides access to the
/// underlying operation and filter used in expanding the output.
/// Furthermore, it reduces the symbol size for an expansion skeleton,
/// hence, reducing the total compilation time.
///
/// @tparam Op        the operation to be used while expanding the input.
/// @tparam Span      the iteration space for elements of the expansion
///                   skeleton
/// @tparam pos_aware whether the computation needs spatial information
///                   for its computation or not.
/// @tparam Filter    the filter to be applied to expand the input.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Span, bool pos_aware,
          typename Filter, bool SetResult>
struct expand_from_pow_two
  : public decltype(
             skeletons::elem<Span>(
               skeletons::expand_from_pow_two_pd<pos_aware, SetResult>(
                 std::declval<Op>(), std::declval<Filter>())
             )
           )
{
  using skeleton_tag_type = tags::expand_from_pow_two;
  using base_type = decltype(
                      skeletons::elem<Span>(
                        skeletons::expand_from_pow_two_pd<
                          pos_aware, SetResult>(
                            std::declval<Op>(), std::declval<Filter>())));

  expand_from_pow_two(Op const& op, Filter const& filter)
    : base_type(
        skeletons::elem<Span>(
          skeletons::expand_from_pow_two_pd<pos_aware, SetResult>(op, filter)
        )
      )
  { }

  Op get_op(void) const
  {
    return base_type::nested_skeleton().get_op();
  }

  Filter get_filter(void) const
  {
    return base_type::nested_skeleton().get_filter();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of an expansion skeleton
/// by exposing only the necessary information in its representation.
///
/// This specialization handles the unfiltered case of expansion in
/// which the expansion simply replicates the input to match the
/// desired size.
///
/// This abstractions not only makes the reconstruction of an
/// expansion skeleton easier, but also provides access to the
/// underlying operation used in expanding the output. Furthermore, it
/// reduces the symbol size for an expansion skeleton, hence, reducing
/// the total compilation time.
///
/// @tparam Op        the operation to be used while expanding the input.
/// @tparam Span      the iteration space for elements of the expansion
///                   skeleton
/// @tparam pos_aware whether the computation needs spatial information
///                   for its computation or not.
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Span, bool pos_aware, bool SetResult>
struct expand_from_pow_two<Op, Span, pos_aware, stapl::use_default, SetResult>
  : public decltype(
             skeletons::elem<Span>(
               skeletons::expand_from_pow_two_pd<pos_aware, SetResult>(
                  std::declval<Op>())
             )
           )
{
  using skeleton_tag_type = tags::expand_from_pow_two;
  using base_type = decltype(
                      skeletons::elem<Span>(
                        skeletons::expand_from_pow_two_pd<
                          pos_aware, SetResult>(
                            std::declval<Op>())));

  expand_from_pow_two(Op const& op, stapl::use_default)
    : base_type(
        skeletons::elem<Span>(
          skeletons::expand_from_pow_two_pd<pos_aware, SetResult>(op)
        )
      )
  { }

  Op get_op(void) const
  {
    return base_type::nested_skeleton().get_op();
  }
};

}


namespace result_of {

template <typename Span,
          bool pos_aware,
          typename Op,
          typename Filter,
          bool SetResult>
using expand_from_pow_two = skeletons_impl::expand_from_pow_two<
                              typename std::decay<Op>::type,
                              stapl::default_type<Span, spans::balanced<>>,
                              pos_aware,
                              typename std::decay<Filter>::type,
                              SetResult>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief This skeleton is used as the post-processing phase of various
/// skeletons including broadcast, n-partition, etc. Basically, this
/// skeleton tries to expand the result of a pow-of-two skeleton to the
/// given non-pow-of-two size.
///
/// @tparam Span      the inner span on which the
///                   @c expand_from_pow_two should be defined
/// @tparam pos_aware whether the op needs to know the position of
///                   the node in the dependence graph or not
/// @param  op        the operation to be used in order to expand the
///                   input size to the given size
/// @param  filter    the filter workfunction to be applied before
///                   expanding to the given size
/// @return a expand from power of two skeleton with given span, filter,
///         position awareness, and operation
///
/// @see broadcast
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename Span   = stapl::use_default,
          bool pos_aware  = false,
          bool SetResult = false,
          typename Op,
          typename Filter = skeletons::no_filter>
result_of::expand_from_pow_two<Span, pos_aware, Op, Filter, SetResult>
expand_from_pow_two(Op&& op, Filter&& filter = Filter())
{
  return result_of::expand_from_pow_two<
           Span, pos_aware, Op, Filter, SetResult>(
             std::forward<Op>(op),
             std::forward<Filter>(filter));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_EXPAND_FROM_POW_TWO_HPP

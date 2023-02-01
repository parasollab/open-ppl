/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_SET_RESULT_HPP
#define STAPL_SKELETONS_FUNCTIONAL_SET_RESULT_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/param_deps/set_result_pd.hpp>
#include <stapl/skeletons/spans/balanced.hpp>
#include <stapl/skeletons/functional/skeleton_traits.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

template <std::size_t Arity, typename Op, typename SkeletonTraits>
struct set_result
  : public decltype(
      skeletons::elem<
        default_type<typename SkeletonTraits::span_type, spans::balanced<1>>>(
        skeletons::set_result_pd<
          Arity,
          default_type<typename SkeletonTraits::span_type, spans::balanced<1>>,
          SkeletonTraits::set_result>(
          std::declval<Op>())))
{
  using skeleton_tag_type = tags::set_result;
  using op_type     = Op;
  static constexpr bool setting_result = SkeletonTraits::set_result;
  using span_t =
    default_type<typename SkeletonTraits::span_type, spans::balanced<1>>;
  using base_type = decltype(
                      skeletons::elem<span_t>(
                        skeletons::set_result_pd<
                          Arity, span_t, setting_result>(
                            std::declval<Op>())));

  set_result(Op const& op, SkeletonTraits const& traits)
    : base_type(skeletons::elem<span_t>(
        skeletons::set_result_pd<Arity, span_t, setting_result>(op)))
  { }

  Op get_op(void) const
  {
    return base_type::nested_skeleton().get_op();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

}

namespace result_of {

template <std::size_t Arity, typename Op, typename SkeletonTraits>
using set_result =
  skeletons_impl::set_result<Arity,
                             typename std::decay<Op>::type,
                             typename std::decay<SkeletonTraits>::type>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief the skeleton which is used to specify which tasks should
///        put their results on the environment result container according
///        to the span which is passed in the @c traits
///
/// @tparam Arity the number of inputs are passed to the skeleton
/// @param op     the workfunction to be used in each @c set_result_pd
///               parametric dependency is spanned or not
/// @param traits the @c skeleton_traits to be used
///
/// @see sink_value
/// @see skeleton_traits
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <std::size_t Arity = 1,
          typename Op,
          typename SkeletonTraits = skeletons_impl::default_skeleton_traits>
result_of::set_result<Arity, Op, SkeletonTraits>
set_result(Op&& op, SkeletonTraits&& traits = SkeletonTraits())
{
  return result_of::set_result<Arity, Op, SkeletonTraits>(
    std::forward<Op>(op), std::forward<SkeletonTraits>(traits));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_SET_RESULT_HPP

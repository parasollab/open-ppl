/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_PRE_BROADCAST_HPP
#define STAPL_SKELETONS_FUNCTIONAL_PRE_BROADCAST_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/param_deps/pre_broadcast_pd.hpp>
#include <stapl/skeletons/spans/balanced.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {


template <std::size_t Arity, typename Op, typename SkeletonTraits>
struct pre_broadcast
  : public decltype(
      skeletons::elem<
        default_type<typename SkeletonTraits::span_type, spans::balanced<1>>>(
        skeletons::pre_broadcast_pd<
          Arity,
          default_type<typename SkeletonTraits::span_type, spans::balanced<1>>,
          SkeletonTraits::set_result>(
          std::declval<Op>())))
{
  using skeleton_tag_type = tags::pre_broadcast;
  using op_type     = Op;
  static constexpr bool setting_result = SkeletonTraits::set_result;
  using span_t =
    default_type<typename SkeletonTraits::span_type, spans::balanced<1>>;
  using base_type = decltype(
                      skeletons::elem<span_t>(
                        skeletons::pre_broadcast_pd<
                          Arity, span_t, setting_result>(
                            std::declval<Op>())));

  pre_broadcast(Op const& op, SkeletonTraits const& traits)
    : base_type(skeletons::elem<span_t>(
        skeletons::pre_broadcast_pd<Arity, span_t, setting_result>(op)))
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

} // namespace skeletons_impl

namespace result_of {

template <std::size_t Arity, typename Op, typename SkeletonTraits>
using pre_broadcast =
  skeletons_impl::pre_broadcast<Arity,
                                typename std::decay<Op>::type,
                                typename std::decay<SkeletonTraits>::type>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief  This skeleton chooses an element from the domain of the input
///         for broadcast when the input domain is non-scalar and we want to
///         broadcast one element from the domain (e.g. last one).
///
/// @tparam Arity   number of inputs to the @c op workfunction
/// @param  op      the workfunction to be used in each pre_broadcast
///                 parametric dependency
/// @param  traits  the traits to be used (default = default_skeleton_traits)
///
/// @ingroup skeletonsFunctional
/// @todo currently this skeleton chooses the last element of the domain
///       for broadcast, in future should be extended to arbitrary element
///       of domain
//////////////////////////////////////////////////////////////////////
template <std::size_t Arity = 1,
          typename Op,
          typename SkeletonTraits = skeletons_impl::default_skeleton_traits>
result_of::pre_broadcast<Arity, Op, SkeletonTraits>
pre_broadcast(Op&& op, SkeletonTraits&& traits = SkeletonTraits())
{
  return result_of::pre_broadcast<Arity, Op, SkeletonTraits>(
    std::forward<Op>(op), std::forward<SkeletonTraits>(traits));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_PRE_BROADCAST_HPP

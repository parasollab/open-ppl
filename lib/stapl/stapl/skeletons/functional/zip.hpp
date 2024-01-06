/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_ZIP_HPP
#define STAPL_SKELETONS_FUNCTIONAL_ZIP_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/param_deps/zip_pd.hpp>
#include <stapl/skeletons/functional/skeleton_traits.hpp>
#include <stapl/skeletons/transformations/transform.hpp>
#include <stapl/skeletons/executors/execution_params.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a filtered zip skeleton
/// by exposing only the necessary information in its representation.
///
/// A filtered zip skeleton is simply a parametric dependency that
/// combines @c Arity elements in each fine-grain computation and
/// applies the filter on the produced result.
///
/// This skeleton is the most used skeleton used in several composed
/// skeletons such as map (which is simply zip<1>), sink, etc.
///
/// This abstraction not only makes the reconstruction of a
/// a zip skeleton easier, but also reduces the symbol size for a
/// zip skeleton, hence, reducing the total compilation time.
///
/// @tparam Arity          the arity of the operation.
/// @tparam Op             the operation to be applied on the input.
/// @tparam SkeletonTraits the traits to be used @see skeleton_traits
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <int Arity, typename Op, typename SkeletonTraits>
struct zip
  : public decltype(
             skeletons::elem<
               default_type<
                 typename SkeletonTraits::span_type, spans::balanced<1>>,
               typename SkeletonTraits::flows_type>(
               skeletons::zip_pd<
                 Arity,
                 default_type<
                   typename SkeletonTraits::span_type, spans::balanced<1>
                 >::dims_num::value,
                 SkeletonTraits::set_result>(
                 std::declval<Op>(),
                 std::declval<typename SkeletonTraits::filter_type>(),
                 std::declval<typename SkeletonTraits::mapper_type>())))
{
  static constexpr bool set_result = SkeletonTraits::set_result;

  using skeleton_tag_type = tags::zip<stapl::use_default, Arity>;
  using op_type           = Op;
  using filter_type       = typename SkeletonTraits::filter_type;

private:
  using span_t            = default_type<
                              typename SkeletonTraits::span_type,
                              spans::balanced<1>>;
  using flows_t           = typename SkeletonTraits::flows_type;
  using mapper_t          = typename SkeletonTraits::mapper_type;
  using base_t            = decltype(
                              skeletons::elem<span_t, flows_t>(
                                skeletons::zip_pd<
                                  Arity, span_t::dims_num::value, set_result>(
                                    std::declval<op_type>(),
                                    std::declval<filter_type>(),
                                    std::declval<mapper_t>())));

public:
  zip(op_type const& op, SkeletonTraits const& traits)
    : base_t(
        skeletons::elem<span_t, flows_t>(
          skeletons::zip_pd<
            Arity, span_t::dims_num::value, set_result>(
              op, traits.get_filter(), traits.get_mapper())))
  { }

  op_type const& get_op(void) const
  {
    return base_t::nested_skeleton().get_op();
  }

  filter_type const& get_filter() const
  {
    return base_t::nested_skeleton().get_filter();
  }

  void define_type(typer& t)
  {
    t.base<base_t>(*this);
  }
};

} // namespace skeletons_impl

namespace result_of {

template <int Arity,
          typename Op,
          typename SkeletonTraits>
using zip = skeletons_impl::zip<
              Arity,
              typename std::decay<Op>::type,
              typename std::decay<SkeletonTraits>::type>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief A filtered zip is similar to @c zip skeleton but it applies
/// a filter function on the producer side before sending data along
/// the edges to each parametric dependency.
///
/// @tparam Arity  the arity of zip
/// @param  op     the workfunction to be used in each zip parametric
///                dependency.
/// @param  traits the traits to be used (default = default_skeleton_traits)
///
/// @see zip
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <int Arity = 2,
          typename Op,
          typename SkeletonTraits = skeletons_impl::default_skeleton_traits,
          typename =
            typename std::enable_if<
              !is_skeleton<typename std::decay<Op>::type>::value>::type>
result_of::zip<Arity, Op, SkeletonTraits>
zip(Op&& op, SkeletonTraits&& traits = SkeletonTraits())
{
  return result_of::zip<Arity, Op, SkeletonTraits>(
          std::forward<Op>(op),
          std::forward<SkeletonTraits>(traits));
}

//////////////////////////////////////////////////////////////////////
/// @brief Creates a zip skeleton over a nested skeleton composition
/// by transforming the inner skeleton to a suitable skeleton for
/// nested execution.
///
/// @tparam Arity           the arity of zip
/// @tparam ExecutionParams execution parameters for the nested section
/// @param  op              the skeleton to be used in the nested section
/// @param  traits          the traits to be used
///                         (default = default_skeleton_traits)
///
/// @see zip
///
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <int Arity = 2,
          typename Op,
          typename SkeletonTraits = skeletons_impl::default_skeleton_traits,
          typename ExecutionParams =
            skeletons_impl::default_execution_params,
          typename = typename std::enable_if<
            is_skeleton<typename std::decay<Op>::type>::value>::type>
result_of::zip<Arity,
               decltype(skeletons::transform<tags::nest>(
                 std::declval<Op>(), std::declval<ExecutionParams>())),
               SkeletonTraits>
zip(Op&& op,
    SkeletonTraits&& traits = SkeletonTraits(),
    ExecutionParams&& execution_params = ExecutionParams())
{
  return skeletons::zip<Arity>(
    skeletons::transform<tags::nest>(
      std::forward<Op>(op),
      std::forward<ExecutionParams>(execution_params)),
    std::forward<SkeletonTraits>(traits));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_ZIP_HPP

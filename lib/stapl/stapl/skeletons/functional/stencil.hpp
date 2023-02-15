/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_STENCIL_HPP
#define STAPL_SKELETONS_FUNCTIONAL_STENCIL_HPP

#include <type_traits>
#include <utility>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/skeletons/utility/skeleton.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include <stapl/skeletons/utility/lightweight_multiarray.hpp>
#include <stapl/skeletons/operators/elem.hpp>
#include <stapl/skeletons/param_deps/stencil_pd.hpp>

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

template <typename Op, typename Filter, typename Tag>
struct stencil;

//////////////////////////////////////////////////////////////////////
/// @brief  The stencil skeleton updates all array elements Using neighboring
///         elements in a fixed pattern (called the stencil),
///         each element of input's new value is computed by applying the given
///         operator on neighbor elements and its current value.
///
/// @tparam Op         the underlying operation to combine the input element.
/// @tparam Filter     the filter to be applied on the result produced by
///                    the operation.
/// @tparam isPeriodic determines how to handle physical boundary conditions.
/// @tparam dim        dimensionality of the stencil computation (1D, 2D, or 3D)
/// @tparam numPoints  number of points used for stencil computation.
///
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template <typename Op, typename Filter,
          bool isPeriodic, int dim, int numPoints>
struct stencil <Op, Filter, tags::stencil<dim, numPoints, isPeriodic>>
  : public decltype(
             skeletons::elem<spans::blocked<dim>>(
               skeletons::stencil_pd<isPeriodic, dim, numPoints>(
                 std::declval<Op>(), std::declval<Filter>()))
           )
{
public:
  using skeleton_tag_type   = tags::stencil<dim, numPoints, isPeriodic>;
  using op_type             = Op;
  using filter_type         = Filter;

  using base_type           =
    decltype(
      skeletons::elem<spans::blocked<dim>>(
        skeletons::stencil_pd<isPeriodic, dim, numPoints>(
          std::declval<Op>(), std::declval<Filter>()))
    );

  stencil(Op const& op, Filter const& filter)
    : base_type(
        skeletons::elem<spans::blocked<dim>>(
          skeletons::stencil_pd<isPeriodic, dim, numPoints>(op, filter))
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


} // namespace skeletons

namespace result_of {

template <typename Tag,
          typename Op,
          typename Filter>
using stencil = skeletons_impl::stencil<
                  typename std::decay<Op>::type,
                  typename std::decay<Filter>::type,
                  stapl::default_type<Tag, tags::stencil<1, 3>>>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief  The stencil skeleton updates all array elements Using
///         neighboring elements in a fixed pattern (called the stencil),
///         each element of input's new value is computed by applying the
///         given operator on neighbor elements and its current value.
/// @tparam Tag        stencil tag that specifies dimension, number of points
///                    and whether boundary conditions are periodic or not.
/// @tparam Op         the underlying operation to combine the input elements.
/// @tparam Filter     the filter to be applied to get the desired
///                    boundary values
/// @ingroup skeletonsFunctional
//////////////////////////////////////////////////////////////////////
template <typename Tag = stapl::use_default,
          typename Op,
          typename Filter = skeletons::no_filter>
result_of::stencil<Tag, Op,  Filter>
stencil(Op&& op, Filter&& filter = Filter())
{
  return result_of::stencil<Tag, Op, Filter>(
           std::forward<Op>(op),
           std::forward<Filter>(filter));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_STENCIL_HPP

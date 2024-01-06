/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_SPANS_SPAN_HELPERS_HPP
#define STAPL_SKELETONS_SPANS_SPAN_HELPERS_HPP

#include <stapl/algorithms/functional.hpp>
#include <stapl/skeletons/utility/utility.hpp>
#include <stapl/utility/tuple/pad_tuple.hpp>
#include <stapl/views/type_traits/is_segmented_view.hpp>
#include <stapl/views/metadata/coarseners/null.hpp>

namespace stapl {

namespace skeletons {
namespace spans {
namespace span_helpers {

//////////////////////////////////////////////////////////////////////
/// @brief Computes size of the view after coarsening for setting the
///        the dimensions needed for nested execution of skeletons.
///
/// @tparam Result     dimension type after coarsening.
/// @tparam Coarsener  type of coarsener is used to coarsen the view
//////////////////////////////////////////////////////////////////////
template <typename Result, typename Coarsener>
struct coarsened_dim
{
  template <typename View>
  static Result apply(View const& view)
  {
    abort("size of the coarsened nested execution should be set manually");
    auto&& dims = view.domain().dimensions();
    constexpr size_t padded_size = tuple_size<Result>::value;
    return tuple_ops::pad_tuple<padded_size>(dims, 1);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for the case the view doesn't need to be
///        coarsened.
///
/// @tparam Result     dimension type after coarsening.
//////////////////////////////////////////////////////////////////////
template <typename Result>
struct coarsened_dim<Result, stapl::null_coarsener>
{
  template <typename View>
  static Result apply(View const& view)
  {
    auto&& dims = view.domain().dimensions();
    constexpr size_t padded_size = tuple_size<Result>::value;
    return tuple_ops::pad_tuple<padded_size>(dims, 1);
  }
};

} // namespace span_helpers

/////////////////////////////////////////////////////////////////////////
/// @brief Handles the base case, the most bottom level.
///        Any coarsening transformation needs to be applied to get the
///        correct dimension of tasks in last level.
////////////////////////////////////////////////////////////////////////
template <int level, typename Op, typename View, typename LevelDims>
void
compute_total_dimension_helper(
  std::false_type, View const& view, LevelDims& level_dims)
{
  using coarsener_t = typename Op::execution_params_t::coarsener_type;

  using result_t = typename LevelDims::value_type;

  level_dims[level_dims.size() - level - 1] =
    span_helpers::coarsened_dim<result_t, coarsener_t>::apply(view);
}

/////////////////////////////////////////////////////////////////////////
/// @brief For the case that View is nested(it's elements are views too)
////////////////////////////////////////////////////////////////////////
template <int level, typename Op, typename View, typename LevelDims>
void
compute_total_dimension_helper(
  std::true_type, View const& view, LevelDims& level_dims)
{
  using is_nested =
    std::integral_constant<bool, 0 != level - 1>;

  constexpr size_t padded_size =
    tuple_size<typename LevelDims::value_type>::value;

  level_dims[level_dims.size() - level - 1] =
    tuple_ops::pad_tuple<padded_size>(view.domain().dimensions(), 1);

  compute_total_dimension_helper<
    level - 1,
    typename Op::wrapped_skeleton_type::op_type>(
    is_nested(),
    view.make_reference(view.domain().first()),
    level_dims);
}

//////////////////////////////////////////////////////////////////////
/// @brief It goes through to the nested views and calculates the
///        flattened equivalent of nested dimensions. For example,
///        for a 2*2 view that each of its elements is 3*3 view
///        the total dimension would be 6*6 and level dims would a
///        vector of tuples([(2,2) , (3,3)]).
///        This information is needed for mappers and currently
///        suffices just for fine grain executions.
///
/// @param view       the input view
/// @param level_dims the variable that is used for storing
///                   the dimension of each nested variable
///
/// @ingroup skeletonsSpans
//////////////////////////////////////////////////////////////////////
template <int level, typename Op, typename View, typename LevelDims>
void
compute_total_dimension(View const& view, LevelDims& level_dims)
{
  using is_nested =
    std::integral_constant<bool, 0 != level - 1>;

  constexpr size_t padded_size =
    tuple_size<typename LevelDims::value_type>::value;

  level_dims[level_dims.size() - level - 1] =
    tuple_ops::pad_tuple<padded_size>(view.domain().dimensions(), 1);

  compute_total_dimension_helper<level - 1, Op>(
    is_nested(),
    view.make_reference(view.domain().first()),
    level_dims);
}

} // namespace spans
} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_SPANS_SPAN_HELPERS_HPP

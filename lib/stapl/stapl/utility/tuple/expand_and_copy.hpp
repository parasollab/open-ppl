/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_EXPAND_AND_COPY_HPP
#define STAPL_UTILITY_EXPAND_AND_COPY_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/utility/tuple/tuple_contains.hpp>
#include <stapl/utility/tuple/homogeneous_tuple.hpp>

namespace stapl {
namespace tuple_ops {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Implementation of @see tuple_ops::expand_and_copy.
///
/// @tparam I The index of the output tuple that we are going to write to
/// @tparam D The total size of the output tuple
/// @tparam J The index of the input tuple that we are going to read from
/// @tparam Tuple The type of the input tuple
/// @tparam ExpandedTuple The type of the output tuple
/// @tparam Slices The tuple of compile-time integrals that specifies which
///                indices to keep or discard
/// @tparam FillValue The value that will be placed in empty positions
//////////////////////////////////////////////////////////////////////
template<int I, int D, int J, typename Tuple, typename ExpandedTuple,
         typename Slices, int FillValue = 0>
struct expand_and_copy_impl
{
  using type = ExpandedTuple;

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Recurse and add the value at index J
  //////////////////////////////////////////////////////////////////////
  template<typename... T>
  static type apply_impl(std::integral_constant<bool, true>,
                         Tuple const& tup, T... t)
  {
    return expand_and_copy_impl<I+1, D, J+1, Tuple, ExpandedTuple,
                                Slices, FillValue>::apply(
      tup, t..., std::get<J>(tup)
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Recurse and add the fill value
  //////////////////////////////////////////////////////////////////////
  template<typename... T>
  static type apply_impl(std::integral_constant<bool, false>,
                         Tuple const& tup, T... t)
  {
    return expand_and_copy_impl<I+1, D, J, Tuple, ExpandedTuple,
                                Slices, FillValue>::apply(
      tup, t..., FillValue
    );
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Expand the tuple and pass along the accumulated result
  /// @param tup The input tuple
  /// @param t A parameter pack of the constituent parts of the output tuple
  //////////////////////////////////////////////////////////////////////
  template<typename... T>
  static type apply(Tuple const& tup, T... t)
  {
    using contains_t = typename tuple_ops::tuple_contains<
                         std::integral_constant<int, I>, Slices>::type;

    // We should push the value at the current index if it appears in Slices
    // Otherwise, we're going to push the fill value
    constexpr bool should_push = contains_t::value;

    return apply_impl(std::integral_constant<bool, should_push>(), tup, t...);
  }
};


template<int I, int J, typename Tuple, typename ExpandedTuple,
         typename Slices, int FillValue>
struct expand_and_copy_impl<I, I, J, Tuple,
                            ExpandedTuple, Slices, FillValue>
{
  using type = ExpandedTuple;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create the final output tuple
  /// @param tup The input tuple
  /// @param t A parameter pack of the constituent parts of the output tuple
  //////////////////////////////////////////////////////////////////////
  template<typename... T>
  static type apply(Tuple const&, T... t)
  {
    return type(t...);
  }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Transforms a d-dimensional tuple to a k-dimensional tuple (k>d)
///        by copying the values of the old tuple and placing them in
///        the larger tuple at indices specified by a tuple of integral
///        constants.
///
///        For example, if the input tuple is (a,b), OutputDims is 4
///        and Slices is <1,3>, then the output would be (0,a,0,b).
///
///        Note that this function currently only supports tuples
///        of homogeneous types.
///
/// @tparam Slices A tuple of std::integral_constants which specifies
///                which indices in the output tuple to copy to.
/// @param  t The tuple to filter
/// @param  fill_value An instance of an integral constant that represents the
///         value that will exist in the positions that were not copied over.
/// @return A tuple of size OutputDims
//////////////////////////////////////////////////////////////////////
template<int OutputDims, typename Slices, typename Tuple, typename FillValue>
typename homogeneous_tuple_type<
    OutputDims,
    typename tuple_element<0, typename std::decay<Tuple>::type>::type
>::type
expand_and_copy(Tuple&& t, FillValue&& fill_value)
{
  using output_type = typename homogeneous_tuple_type<
    OutputDims,
    typename stapl::tuple_element<0, typename std::decay<Tuple>::type>::type
  >::type;

  constexpr int fill = FillValue::value;

  using f = detail::expand_and_copy_impl<
    0, OutputDims, 0, Tuple, output_type, Slices, fill
  >;

  return f::apply(std::forward<Tuple>(t));
}

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_EXPAND_AND_COPY_HPP

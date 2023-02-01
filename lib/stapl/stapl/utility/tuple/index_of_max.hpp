/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTIILTY_TUPLE_INDEX_OF_MAX_HPP
#define STAPL_UTIILTY_TUPLE_INDEX_OF_MAX_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>

#include <stapl/utility/tuple/tuple_contains.hpp>
#include <stapl/utility/tuple/homogeneous_tuple.hpp>

namespace stapl {
namespace tuple_ops {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper metafunction to compute the index of maximum value.
///
/// @tparam Tuple The tuple of compile time constants
/// @tparam I Current index we are iterating on
/// @tparam Max Max value that we have seen so far
/// @tparam MaxIndex Index of the max value that we have seen so far
//////////////////////////////////////////////////////////////////////
template<typename Tuple, int I, std::size_t Max, std::size_t MaxIndex>
struct index_of_max_impl
{
  static constexpr std::size_t value = std::conditional<
    /* if */    std::tuple_element<I, Tuple>::type::value <= Max,
    /* then */  index_of_max_impl<Tuple, I-1, Max, MaxIndex>,
    /* else */  index_of_max_impl<Tuple, I-1,
                  std::tuple_element<I, Tuple>::type::value, I>
    >::type::value;
};

template<typename Tuple, std::size_t Max, std::size_t MaxIndex>
struct index_of_max_impl<Tuple, -1, Max, MaxIndex>
{
  static constexpr std::size_t value = MaxIndex;
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute the index of the maximum integral
///        value in a compile-time tuple.
///
///        For example, given the input tuple<
///          integral_constant<int, 0>,
///          integral_constant<int, 2>,
///          integral_constant<int, 1>>
///        would be the value 1, as the max value (2) is at index 1.
///
/// @tparam Tuple The tuple of compile time constants
//////////////////////////////////////////////////////////////////////
template<typename Tuple>
struct index_of_max
{
  static constexpr std::size_t value = detail::index_of_max_impl<
      Tuple,
      std::tuple_size<Tuple>::type::value-1,
      std::tuple_element<
        std::tuple_size<Tuple>::type::value-1, Tuple
      >::type::value,
      std::tuple_size<Tuple>::type::value-1
    >::value;

  using type = std::integral_constant<std::size_t, value>;
};

} // namespace tuple_ops

} // namespace stapl

#endif

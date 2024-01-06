/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_FROM_ARRAY_HPP
#define STAPL_UTILITY_TUPLE_FROM_ARRAY_HPP

#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/utility/tuple/homogeneous_tuple.hpp>

namespace stapl {
namespace tuple_ops {

template<typename Array,
         typename Indices =
           make_index_sequence<stapl::tuple_size<Array>::value>>
struct from_array_impl;

//////////////////////////////////////////////////////////////////////
/// @brief Convert an std::array to a tuple of homogeneous types
//////////////////////////////////////////////////////////////////////
template<typename Array, std::size_t... Indices>
struct from_array_impl<Array, index_sequence<Indices...>>
{
  using type = typename homogeneous_tuple_type<
    stapl::tuple_size<Array>::value, typename Array::value_type
  >::type;

  static type apply(Array const& a)
  {
    return std::make_tuple(a[Indices]...);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Convert an std::array to a tuple of homogeneous types.
///
/// @param a an std::array
//////////////////////////////////////////////////////////////////////
template<typename Array>
typename from_array_impl<Array>::type
from_array(Array const& a)
{
  return from_array_impl<Array>::apply(a);
}

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_FROM_ARRAY_HPP

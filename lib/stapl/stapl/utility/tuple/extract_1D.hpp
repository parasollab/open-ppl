/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_EXTRACT_1D_HPP
#define STAPL_UTILITY_TUPLE_EXTRACT_1D_HPP

#include <type_traits>

#include <stapl/utility/utility.hpp>
#include "tuple.hpp"

namespace stapl {
namespace tuple_ops {

template <typename T, std::size_t sz = tuple_size<T>::value>
struct extract_1D_impl
{
  T static apply(T const& t)
  {
    return t;
  }
};

template <typename T>
struct extract_1D_impl<T, 1>
{
  typename tuple_element<0, T>::type
  static apply(T const& t)
  {
    return get<0>(t);
  }
};

template <typename T>
auto extract_1D(T&& t) -> decltype(
  extract_1D_impl<typename std::decay<T>::type>::apply(std::forward<T>(t)))
{
  return extract_1D_impl<typename std::decay<T>::type>::apply(
    std::forward<T>(t));
}

} // namespace tuple_ops

} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_EXTRACT_1D_HPP

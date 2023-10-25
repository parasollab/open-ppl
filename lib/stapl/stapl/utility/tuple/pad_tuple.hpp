/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_PAD_TUPLE_HPP
#define STAPL_UTILITY_TUPLE_PAD_TUPLE_HPP

#include <type_traits>

#include <stapl/utility/utility.hpp>
#include "tuple.hpp"

namespace stapl {
namespace tuple_ops {

template<std::size_t N, bool is_size_t>
struct apply_pad_tuple
{
  template<typename T>
  static auto apply(T&& t, size_t val)
    -> decltype(
        stapl::tuple_cat(homogeneous_tuple<N -1>(val), stapl::make_tuple(t)))
  {
    return stapl::tuple_cat(homogeneous_tuple<N -1>(val), stapl::make_tuple(t));
  }
};

template<std::size_t N>
struct apply_pad_tuple<N, false>
{
  template<typename T>
  static auto apply(T&& t, size_t val)
    -> decltype(
        stapl::tuple_cat(
          homogeneous_tuple<
            N - stapl::tuple_size<typename std::decay<T>::type>::value>(val),t))
  {
    return stapl::tuple_cat(
             homogeneous_tuple<
               N - stapl::tuple_size<typename std::decay<T>::type>::value>(val),
             t);
  }
};

template <std::size_t N, typename T>
auto
pad_tuple(T&& t, std::size_t val)
 -> decltype(
      apply_pad_tuple<
        N,
        std::is_same<
          std::size_t,
          typename std::decay<T>::type>::value>::apply(std::forward<T>(t), val))
{
  return apply_pad_tuple<
    N,
    std::is_same<std::size_t, typename std::decay<T>::type>::value
  >::apply(std::forward<T>(t), val);
}

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_PAD_TUPLE_HPP

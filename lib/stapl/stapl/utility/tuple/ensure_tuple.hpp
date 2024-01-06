/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_ENSURE_TUPLE_HPP
#define STAPL_UTILITY_TUPLE_ENSURE_TUPLE_HPP

#include <type_traits>

#include <stapl/utility/utility.hpp>
#include "tuple.hpp"

namespace stapl {
namespace tuple_ops {
namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that wraps a non-tuple type into a tuple of
///        size one. If the type is already a tuple, it acts as an
///        identity.
///
///        For example, ensure_tuple<char> would return tuple<char> but
///        ensure_tuple<tuple<char,float>> would return tuple<char,float>.
///
/// @tparam T The type to obtain the tuple from.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct ensure_tuple
{
  using type = stapl::tuple<T>;

  template<typename U>
  static type apply(U&& u)
  {
    return std::forward_as_tuple(std::forward<U>(u));
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization when T is already a tuple.
//////////////////////////////////////////////////////////////////////
template<typename... Args>
struct ensure_tuple<stapl::tuple<Args...>>
{
  using type = stapl::tuple<Args...>;

  static type apply(type const& t)
  {
    return t;
  }
};

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief Always return a tuple representation of given argument.
///        If it is already a tuple, return it unchanged.
///
/// @param tup Object to obtain the tuple from.
//////////////////////////////////////////////////////////////////////
template<typename T>
auto ensure_tuple(T&& tup)
STAPL_AUTO_RETURN (
  result_of::ensure_tuple<typename std::decay<T>::type>::apply(
    std::forward<T>(tup))
)

template<typename T>
using ensure_tuple_t = typename tuple_ops::result_of::ensure_tuple<T>::type;

} // namespace tuple_ops

} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_ENSURE_TUPLE_HPP

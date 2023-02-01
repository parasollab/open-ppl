/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_FILTER_TYPE_HPP
#define STAPL_UTILITY_TUPLE_FILTER_TYPE_HPP

#include "tuple.hpp"
#include "push_front.hpp"

namespace stapl {
namespace tuple_ops {

//////////////////////////////////////////////////////////////////////
/// @brief Filter the types in a tuple based on a predicate metafunction.
//////////////////////////////////////////////////////////////////////
template<class Tuple, template<class...> class F, class... Args>
struct filter_types;

template<template<class...> class F, class... Args>
struct filter_types<tuple<>, F, Args...>
{
  using type = tuple<>;

  static type call(type const& t)
  {
    return t;
  }

  template<size_t I, class T>
  static type apply(T const&)
  {
    return {};
  }
};

namespace detail {

template<bool ShouldPush, typename Tuple, typename>
struct push_type_if
{
  using type = Tuple;

  template<class T>
  static type call(Tuple const& tuple, T const&)
  {
    return tuple;
  }
};

template<typename Tuple, typename T>
struct push_type_if<true, Tuple, T>
  : result_of::push_front<Tuple, T>
{ };

} // namespace detail

template<class T, class... Ts, template<class...> class F,
  class... Args>
struct filter_types<tuple<T, Ts...>, F, Args...>
{
private:
  using recursive = filter_types<tuple<Ts...>, F, Args...>;
  static constexpr bool keep = F<T, Args...>::value;
  using push = detail::push_type_if<keep, typename recursive::type, T>;

public:
  using type = typename push::type;

  template<class Tuple>
  inline static type call(Tuple const& t)
  {
    return apply<0>(t);
  }

  template<size_t I, class Tuple>
  inline static type apply(Tuple const& tuple)
  {
    return push::call(recursive::template apply<I+1>(tuple), get<I>(tuple));
  }
};

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_FILTER_TYPE_HPP

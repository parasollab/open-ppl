/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_FOLD_TYPE_HPP
#define STAPL_UTILITY_TUPLE_FOLD_TYPE_HPP

#include "tuple.hpp"
namespace stapl {
namespace tuple_ops {

//////////////////////////////////////////////////////////////////////
/// @brief Perform a left fold over the types in a tuple.
//////////////////////////////////////////////////////////////////////
template<template<class...> class F, class T, class Tuple, class... Args>
struct fold_types;

template<template<class...> class F, class T, class... Args>
struct fold_types<F, T, tuple<>, Args...>
{
  using type = T;
};

template<template<class...> class F, class T, class T0, class... Ts,
  class... Args>
struct fold_types<F, T, tuple<T0, Ts...>, Args...>
  : fold_types<F, typename F<T, T0, Args...>::type, tuple<Ts...>, Args...>
{ };

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_FOLD_TYPE_HPP

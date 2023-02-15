/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_INSERT_TYPE_HPP
#define STAPL_UTILITY_TUPLE_INSERT_TYPE_HPP

#include "tuple.hpp"
namespace stapl {
namespace tuple_ops {

//////////////////////////////////////////////////////////////////////
/// @brief Push back a type into a tuple only if it's not already there.
//////////////////////////////////////////////////////////////////////
template<class Tuple, class T>
using insert_type = std::conditional<
  tuple_contains_type<T, Tuple>::value,
  Tuple,
  typename result_of::push_back<Tuple, T>::type
>;

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_INSERT_TYPE_HPP

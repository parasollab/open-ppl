/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_TYPE_TRAITS_CONTAINER_LEVELS_HPP
#define STAPL_CONTAINERS_TYPE_TRAITS_CONTAINER_LEVELS_HPP

#include "is_container.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine how many containers are nested within
/// the value_types of a given type.
///
/// For example, given the type <t>array<array<vector<int>>></tt>, this
/// metafunction would reflect a value of @c 3.
///
/// @tparam T The type in question
//////////////////////////////////////////////////////////////////////
template<typename T, bool = is_container<T>::value>
struct container_levels
{
  typedef unsigned int value_type;

  static const value_type value = 0;
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_levels for the case when @p T is a
///        container, effectively adding one more to the running total.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct container_levels<T, true>
{
  typedef unsigned int value_type;

  static const value_type value =
    (1 + container_levels<typename T::value_type>::value);
};

} // namespace stapl

#endif

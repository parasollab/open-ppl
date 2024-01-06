/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_CONDITONAL_LOCK_GUARD_HPP
#define STAPL_UTILITY_CONDITONAL_LOCK_GUARD_HPP

#include <stapl/utility/empty_class.hpp>

#include <mutex>
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Helper class that conditionally inherits from another class
///        based on a compile-time condition.
///
///        If the condition is false, then this class is an empty class
///        with no methods.
///
//// @tparam Base Class to inherit
//// @tparam Condition Whether or not to inherit from the class
//////////////////////////////////////////////////////////////////////
template<typename Base, bool Condition>
struct conditonal_inherit
  : std::conditional<Condition, Base, empty_class>::type
{
  using base_type =
    typename std::conditional<Condition, Base, empty_class>::type;

  template<typename... T>
  conditonal_inherit(T&&... args)
    : base_type(std::forward<T>(args)...)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief A lock guard that locks based on a compile-time condition.
///        If the condition is false, then no locking happens.
///
//// @tparam Condition Whether or not to lock
//////////////////////////////////////////////////////////////////////
template<typename Mutex, bool Condition>
using conditional_lock_guard =
  conditonal_inherit<std::lock_guard<Mutex>, Condition>;

} // namespace stapl

#endif

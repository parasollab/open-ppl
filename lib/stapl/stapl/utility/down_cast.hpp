/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_DOWN_CAST_HPP
#define STAPL_UTILITY_DOWN_CAST_HPP

#include <stapl/runtime/stapl_assert.hpp>
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Perform downcast of a pointer from a base class to a derived class.
///   In debug mode, use dynamic_cast and test with assertion.  Otherwise, use
///   static_cast<>().
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template<typename DerivedPtr, typename Base>
inline DerivedPtr down_cast(Base* const base_ptr)
{
  static_assert(std::is_pointer<DerivedPtr>::value,
                "invalid down_cast conversion");

#ifndef STAPL_NDEBUG
  if (base_ptr == nullptr)
    return nullptr;

  DerivedPtr const derived_ptr = dynamic_cast<DerivedPtr>(base_ptr);

  stapl_assert(derived_ptr != nullptr,
    "dynamic_cast downcast to base_ptr failed");
  return derived_ptr;
#else
  return static_cast<DerivedPtr>(base_ptr);
#endif
}


//////////////////////////////////////////////////////////////////////
/// @brief Perform downcast of a reference from a base class to a derived class.
///   In debug mode, use dynamic_cast and test with assertion.  Otherwise, use
///   static_cast<>().
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template<typename Derived, typename Base>
inline Derived& down_cast(Base& base_ref)
{
#ifndef STAPL_NDEBUG
  return dynamic_cast<Derived&>(base_ref);
#else
  return static_cast<Derived&>(base_ref);
#endif
}


//////////////////////////////////////////////////////////////////////
/// @brief Perform downcast of a const reference from a base class to a derived
///   class In debug mode, use dynamic_cast and test with assertion.  Otherwise,
///   use static_cast<>().
/// @ingroup utility
//////////////////////////////////////////////////////////////////////
template<typename Derived, typename Base>
inline Derived const& down_cast(Base const& base_ref)
{
#ifndef STAPL_NDEBUG
  return dynamic_cast<Derived const&>(base_ref);
#else
  return static_cast<Derived const&>(base_ref);
#endif
}

} // namespace stapl

#endif // STAPL_UTILITY_DOWN_CAST_HPP

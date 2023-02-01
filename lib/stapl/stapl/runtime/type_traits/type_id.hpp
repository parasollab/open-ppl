/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_TYPE_ID_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_TYPE_ID_HPP

#include <cstdint>
#include <type_traits>

namespace stapl {

/// Id for a type.
using type_id = std::uintptr_t;


////////////////////////////////////////////////////////////////////
/// @brief Returns the type id of a non-polymorphic type.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T, bool = std::has_virtual_destructor<T>::value>
struct type_id_impl
{
  static constexpr type_id get(void) noexcept
  { return reinterpret_cast<type_id>(static_cast<type_id(*)(void)>(&get)); }

  static constexpr type_id get(T const&) noexcept
  { return get(); }
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref type_id_impl for polymorphic types.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
struct type_id_impl<T, true>
{
  static constexpr type_id get(void) noexcept
  { return T::polymorphic_type_id(); }

  static type_id get(T const& t) noexcept
  { return t.get_polymorphic_type_id(); }
};


////////////////////////////////////////////////////////////////////
/// @brief Returns the type id of the given object.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
constexpr type_id get_type_id(T const& t) noexcept
{
  using U = typename std::remove_cv<T>::type;
  return type_id_impl<U>::get(const_cast<U const&>(t));
}


////////////////////////////////////////////////////////////////////
/// @brief Returns the type id of the given type.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
type_id get_type_id(void) noexcept
{
  return type_id_impl<typename std::remove_cv<T>::type>::get();
}

} // namespace stapl

#endif

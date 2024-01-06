/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_P_OBJECT_PTR_CAST_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_P_OBJECT_PTR_CAST_HPP

#include "has_virtual_base.hpp"
#include "no_access_check_cast.hpp"
#include <type_traits>

namespace stapl {

class p_object;


namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Casts the @c void pointer to pointer to @p T.
///
/// @related p_object_ptr_cast()
/// @ingroup runtimeTypeTraitsImpl
//////////////////////////////////////////////////////////////////////
template<typename T, typename = void>
struct p_object_ptr_cast_impl
{
  static T* apply(void* p) noexcept
  { return static_cast<T*>(p); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref p_object_ptr_cast_impl for @p T that derives
///        from @ref p_object and does not define @c T::has_virtual_base.
///
/// @related p_object_ptr_cast()
/// @ingroup runtimeTypeTraitsImpl
//////////////////////////////////////////////////////////////////////
template<typename T>
struct p_object_ptr_cast_impl<T,
                              typename std::enable_if<
                                std::is_base_of<p_object, T>::value &&
                                !has_virtual_base<T>::value
                              >::type>
{
  static T* apply(void* p) noexcept
  { return no_access_check_cast<T*>(static_cast<p_object*>(p)); }
};


//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref p_object_ptr_cast_impl for @p T that derives
///        from @ref p_object and defines @c T::has_virtual_base.
///
/// @related p_object_ptr_cast()
/// @ingroup runtimeTypeTraitsImpl
///
/// @todo If we had has_virtual_base (or is a virtual base between p_object and
///       T) we can specialize this to allow static_cast if not. We're forced to
///       dynamic_cast because of virtual inheritance.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct p_object_ptr_cast_impl<T,
                              typename std::enable_if<
                                std::is_base_of<p_object, T>::value &&
                                has_virtual_base<T>::value
                              >::type>
{
  static T* apply(void* p) noexcept
  { return dynamic_cast<T*>(static_cast<p_object*>(p)); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Casts the @c void pointer to @p T, where @p T is a distributed
///        object.
///
/// @ingroup runtimeTypeTraits
//////////////////////////////////////////////////////////////////////
template<typename T>
T* p_object_ptr_cast(void* p) noexcept
{
  return p_object_ptr_cast_impl<T>::apply(p);
}

} // namespace runtime

} // namespace stapl

#endif

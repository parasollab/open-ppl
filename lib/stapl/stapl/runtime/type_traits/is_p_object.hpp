/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_IS_P_OBJECT_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_IS_P_OBJECT_HPP

#include <type_traits>

namespace stapl {

class p_object;


////////////////////////////////////////////////////////////////////
/// @brief Detects if @p T is a @ref p_object.
///
/// @see p_object
/// @ingroup ARMITypeTraits
///
/// @todo Use @c object_traits class to detect if @p T is a @ref p_object.
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_p_object
: public std::is_base_of<p_object, typename std::remove_cv<T>::type>
{ };



////////////////////////////////////////////////////////////////////
/// @brief Detects if @p T is a reference to a @ref p_object.
///
/// @see p_object
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_p_object_reference
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_p_object_reference for references.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_p_object_reference<T&>
: public is_p_object<T>::type
{ };



////////////////////////////////////////////////////////////////////
/// @brief Detects if @p T is a pointer to a @ref p_object.
///
/// @see p_object
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_p_object_pointer
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_p_object_pointer for pointers.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_p_object_pointer<T*>
: public is_p_object<T>::type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_p_object_pointer for @c const pointers.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_p_object_pointer<T* const>
: public is_p_object<T>::type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_p_object_pointer for @c volatile pointers.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_p_object_pointer<T* volatile>
: public is_p_object<T>::type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_p_object_pointer for @c const @c volatile
///        pointers.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_p_object_pointer<T* const volatile>
: public is_p_object<T>::type
{ };

} //  namespace stapl

#endif

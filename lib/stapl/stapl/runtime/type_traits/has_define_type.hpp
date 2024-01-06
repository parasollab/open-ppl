/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_HAS_DEFINE_TYPE_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_HAS_DEFINE_TYPE_HPP

#include "../serialization/typer_fwd.hpp"
#include <type_traits>
#include <utility>

namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Returns @c std::false_type if @c T::define_type(stapl::typer&) does
///        not exist or is not accessible.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
constexpr std::false_type
define_type_exists(...)
{ return std::false_type{}; }


////////////////////////////////////////////////////////////////////
/// @brief Returns @c std::true_type if @c T::define_type(stapl::typer&) exists.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
constexpr std::true_type
define_type_exists(T* t,
                   decltype(
                     runtime::define_type_cast(*t).define_type(
                       std::declval<typer&>())
                   )* = nullptr)
{ return std::true_type{}; }


////////////////////////////////////////////////////////////////////
/// @brief Detects if @p T has a function @c T::define_type(stapl::typer&).
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T,
         bool = (std::is_scalar<T>::value || std::is_array<T>::value)>
struct has_define_type_impl
: public decltype(define_type_exists(static_cast<T*>(nullptr)))
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref has_define_type_impl for scalars and arrays.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
struct has_define_type_impl<T, true>
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @brief Detects if @p T has a function @c T::define_type(stapl::typer&).
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct has_define_type
: public has_define_type_impl<typename std::remove_cv<T>::type>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref has_define_type for references.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct has_define_type<T&>
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref has_define_type for pointers.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct has_define_type<T*>
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref has_define_type for @c const pointers.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct has_define_type<T* const>
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref has_define_type for @c volatile pointers.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct has_define_type<T* volatile>
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref has_define_type for @c const @c volatile
///        pointers.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct has_define_type<T* const volatile>
: public std::false_type
{ };

} // namespace runtime

} // namespace stapl

#endif

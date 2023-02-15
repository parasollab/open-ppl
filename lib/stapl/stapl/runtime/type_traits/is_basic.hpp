/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_IS_BASIC_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_IS_BASIC_HPP

#include "has_member_types.hpp"
#include <atomic>
#include <type_traits>

namespace std {

template<typename T, std::size_t Size>
struct array;

template<typename T1, typename T2>
struct pair;

template<typename... Types>
class tuple;

} // namespace std


namespace stapl {

template<typename T>
struct is_basic;

template<typename... T>
struct are_basic;


////////////////////////////////////////////////////////////////////
/// @brief Detects known basic types.
///
/// Known basic types are
/// -# fundamental: integral, floating point and void types
/// -# enums
/// -# empty classes
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_known_basic
: public std::integral_constant<
           bool,
           (std::is_fundamental<T>::value ||
            std::is_enum<T>::value        ||
            std::is_empty<T>::value)
          >
{ };



////////////////////////////////////////////////////////////////////
/// @brief Returns if @p T is a basic type.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T, bool, bool>
struct is_basic_impl_or
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_basic_impl_or for known basic types.
///
/// @see is_know_basic
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T, bool B>
struct is_basic_impl_or<T, true, B>
: public std::true_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_basic_impl_or when @c T::member_types is
///        defined.
///
/// @c T::member_types has to be an @c std::tuple and then the specialization of
/// @c std::tuple of @ref is_basic is used.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_basic_impl_or<T, false, true>
: public is_basic<typename T::member_types>
{ };


#if 0
// Uncomment to force compile-time error if member_types is not typedef-ed.
template<typename T>
struct is_basic_impl_or<T, false, false>
{ };
#endif



////////////////////////////////////////////////////////////////////
/// @brief Detects if a type is basic or not.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_basic_impl
: public is_basic_impl_or<
           T,
           is_known_basic<T>::value,
           runtime::has_member_types<T>::value
         >
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_basic_impl for arrays.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T, std::size_t Size>
struct is_basic_impl<T[Size]>
: public is_basic<T>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_basic_impl for arrays with no known size.
///
/// @ingroup runtimeTypeTraitsImpl
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_basic_impl<T[]>
: public is_basic<T>
{ };



////////////////////////////////////////////////////////////////////
/// @brief Returns if @p T is a basic type.
///
/// A basic type is similar to a POD in the sense that it can be copied
/// correctly using @c memcpy().
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_basic
: public is_basic_impl<T>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_basic for @c const types.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_basic<const T>
: public is_basic<T>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_basic for @c volatile types.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_basic<volatile T>
: public is_basic<T>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_basic for @c const @c volatile types.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_basic<const volatile T>
: public is_basic<T>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_basic for references.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_basic<T&>
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_basic for pointers.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_basic<T*>
: public std::is_function<T>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_basic for pointer to member functions.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T, typename U>
struct is_basic<U T::*>
: public std::true_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_basic for @c std::array.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T, std::size_t Size>
struct is_basic<std::array<T, Size>>
: public is_basic<T>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_basic for @c std::pair.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T1, typename T2>
struct is_basic<std::pair<T1, T2>>
: public std::integral_constant<
           bool,
           (is_basic<T1>::value && is_basic<T2>::value)
         >
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_basic for @c std::tuple.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename... T>
struct is_basic<std::tuple<T...>>
: public are_basic<T...>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_basic for @c std::atomic<T>.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_basic<std::atomic<T>>
: public is_basic<T>
{ };



////////////////////////////////////////////////////////////////////
/// @brief Returns if the types @p T are basic types.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename... T>
struct are_basic;


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref are_basic for empty list of types.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<>
struct are_basic<>
: public std::true_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref are_basic for a list of types.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename H, typename... T>
struct are_basic<H, T...>
: public std::integral_constant<
           bool,
           is_basic<H>::value && are_basic<T...>::value
         >
{ };

} // namespace stapl


////////////////////////////////////////////////////////////////////
/// @brief Declares @p T as a basic type.
///
/// @see is_basic
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
#define STAPL_IS_BASIC_TYPE(T)    \
namespace stapl {                 \
                                  \
template<>                        \
struct is_basic<T>                \
: public std::true_type           \
{ };                              \
                                  \
}

#endif

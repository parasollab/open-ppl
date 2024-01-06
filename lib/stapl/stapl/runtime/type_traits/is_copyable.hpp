/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_IS_COPYABLE_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_IS_COPYABLE_HPP

#include "is_basic.hpp"
#include <type_traits>

namespace std {

template<typename T1, typename T2>
struct pair;

template<typename... Types>
class tuple;

template<typename T>
class reference_wrapper;

} // namespace std


namespace boost {

template<typename T>
class reference_wrapper;

} // namespace boost


namespace stapl {

namespace runtime {

////////////////////////////////////////////////////////////////////
/// @brief Typedefs member @c type to @c std::true_type if @p T is a copyable
///        type, otherwise it typedefs it to @c std::false_type.
///
/// A copyable type is not the same as a copy constructible type. Copyable types
/// are types that there is benefit in copying them when communicating in the
/// same address space and they do not introduce race-conditions.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_copyable
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_copyable for @c const types.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_copyable<const T>
: public is_copyable<T>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_copyable for @c volatile types.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_copyable<volatile T>
: public is_copyable<T>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_copyable for @c const @c volatile types.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_copyable<const volatile T>
: public is_copyable<T>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_copyable for @c std::pair.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T1, typename T2>
struct is_copyable<std::pair<T1, T2>>
: public std::integral_constant<
           bool,
           (is_copyable<T1>::value || is_copyable<T2>::value)
         >
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_copyable for non-empty tuple.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename H, typename... T>
struct is_copyable<std::tuple<H, T...>>
: public std::integral_constant<
           bool,
           (is_copyable<H>::value || is_copyable<std::tuple<T...>>::value)
         >
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_copyable for @c std::reference_wrapper that
///        declares it as non-movable.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_copyable<std::reference_wrapper<T>>
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_copyable for @c boost::reference_wrapper
///        that declares it as non-movable.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_copyable<boost::reference_wrapper<T>>
: public std::false_type
{ };



////////////////////////////////////////////////////////////////////
/// @brief Tag type for objects that for which @ref is_copyable is
///        @c std::true_type.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct copyable
{
  using type = T;
};

} // namespace runtime

} // namespace stapl

#endif

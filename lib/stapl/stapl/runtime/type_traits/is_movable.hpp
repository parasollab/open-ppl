/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_IS_MOVABLE_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_IS_MOVABLE_HPP

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
/// @brief Typedefs member @c type to @c std::true_type if @p T is a movable
///        type, otherwise it typedefs it to @c std::false_type.
///
/// A movable type is not the same as a move constructible type. Movable types
/// are types that there is benefit in moving them when communicating in the
/// same address space and they do not introduce race-conditions.
///
/// As such, basic or trivial types, types with an implicitly or explicitly
/// deleted move constructor or types that have a pointer to a @ref p_object are
/// not considered movable.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_movable
: public std::integral_constant<
           bool,
           (!is_basic<T>::value                  &&
            std::is_move_constructible<T>::value &&
            !std::is_trivial<T>::value)
         >
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_movable for references.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_movable<T&>
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_movable for @c const types.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_movable<const T>
: public is_movable<T>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_movable for @c volatile types.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_movable<volatile T>
: public is_movable<T>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_movable for @c const @c volatile types.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_movable<const volatile T>
: public is_movable<T>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_movable for @c std::pair.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T1, typename T2>
struct is_movable<std::pair<T1, T2>>
: public std::integral_constant<
           bool,
           (is_movable<T1>::value || is_movable<T2>::value)
         >
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_movable for non-empty tuple.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename H, typename... T>
struct is_movable<std::tuple<H, T...>>
: public std::integral_constant<
           bool,
           (is_movable<H>::value || is_movable<std::tuple<T...>>::value)
         >
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_movable for @c std::reference_wrapper that
///        declares it as non-movable.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_movable<std::reference_wrapper<T>>
: public std::false_type
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref is_movable for @c boost::reference_wrapper
///        that declares it as non-movable.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_movable<boost::reference_wrapper<T>>
: public std::false_type
{ };



////////////////////////////////////////////////////////////////////
/// @brief Tag type for objects that for which @ref is_movable is
///        @c std::true_type.
///
/// @ingroup runtimeTypeTraits
////////////////////////////////////////////////////////////////////
template<typename T>
struct movable
{
  using type = T;
};

} // namespace runtime

} // namespace stapl

#endif

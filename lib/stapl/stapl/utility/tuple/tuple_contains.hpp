/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_TUPLE_CONTAINS_HPP
#define STAPL_UTILITY_TUPLE_TUPLE_CONTAINS_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>

namespace stapl {
namespace tuple_ops {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that computes whether an integral type
///        appears in a tuple of integral types
///
/// @tparam T The type to search for
/// @tparam Tuple The tuple to search in
//////////////////////////////////////////////////////////////////////
template <typename T, typename Tuple>
struct tuple_contains;

template <typename T, typename Head, typename... Tail>
struct tuple_contains<T, tuple<Head, Tail...>>
 : public std::integral_constant<bool, T::value == Head::value ||
            tuple_contains<T, tuple<Tail...>>::value>
{ };

template<typename T>
struct tuple_contains<T, tuple<>>
 : public std::false_type
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that computes if a type appears in a tuple
///
/// @tparam T The type to search for
/// @tparam Tuple The tuple to search in
//////////////////////////////////////////////////////////////////////
template <typename T, typename Tuple>
struct tuple_contains_type;

template <typename T, typename... Tail>
struct tuple_contains_type<T, tuple<T, Tail...>>
  : public std::true_type
{ };

template <typename T, typename Head, typename... Tail>
struct tuple_contains_type<T, tuple<Head, Tail...>>
  : public tuple_contains_type<T, tuple<Tail...>>
{ };

template<typename T>
struct tuple_contains_type<T, tuple<>>
  : public std::false_type
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that computes if a type does not appear in a tuple
///
/// @tparam T The type to search for
/// @tparam Tuple The tuple to search in
//////////////////////////////////////////////////////////////////////
template<typename T, typename Tuple>
struct not_tuple_contains_type :
  std::integral_constant<bool,
    not tuple_contains_type<T, Tuple>::value>
{ };

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_TUPLE_CONTAINS_HPP

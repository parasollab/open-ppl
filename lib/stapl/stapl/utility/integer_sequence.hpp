/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_UTILITY_INTEGER_SEQUENCE_HPP
#define STAPL_UTILITY_INTEGER_SEQUENCE_HPP

#include <cstddef>
#include <type_traits>

namespace stapl {

////////////////////////////////////////////////////////////////////
/// @brief A compile-time sequence of integers.
///
/// This is implemented in C++14 and described in
/// http://open-std.org/jtc1/sc22/wg21/docs/papers/2013/n3658.html
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<typename T, T... Ints>
struct integer_sequence
{
  typedef T value_type;

  static constexpr std::size_t size(void) noexcept
  { return sizeof...(Ints); }
};


////////////////////////////////////////////////////////////////////
/// @brief Helper alias template for @ref integer_sequence for @c std::size_t.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<std::size_t... Ints>
using index_sequence = integer_sequence<std::size_t, Ints...>;


////////////////////////////////////////////////////////////////////
/// @brief Implementation of @ref make_index_sequence.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<std::size_t N, std::size_t... Ints>
struct make_index_sequence_impl
{
  typedef typename make_index_sequence_impl<N-1, N-1, Ints...>::type type;
};

////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_index_sequence_impl to end the
///        recursion.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<std::size_t... Ints>
struct make_index_sequence_impl<0, Ints...>
{
  typedef index_sequence<Ints...> type;
};


////////////////////////////////////////////////////////////////////
/// @brief Creates an @ref index_sequence with the values
///        <tt>0, 1, ..., N-1</tt>.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<std::size_t N>
using make_index_sequence = typename make_index_sequence_impl<N>::type;


////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute whether or not a type is an index_sequence.
///
/// @ingroup utility
////////////////////////////////////////////////////////////////////
template<typename T>
struct is_index_sequence : std::false_type {};

template<std::size_t... Indices>
struct is_index_sequence<index_sequence<Indices...>> : std::true_type {};

//////////////////////////////////////////////////////////////////////
/// @brief Get the I'th element in an @see integer_sequence.
//////////////////////////////////////////////////////////////////////
template<std::size_t I, class Sequence>
struct sequence_element;

template<class T, T Head, T... Tail>
struct sequence_element<0, integer_sequence<T, Head, Tail...>>
  : std::integral_constant<T, Head>
{ };

template<std::size_t I, class T, T Head, T... Tail>
struct sequence_element<I, integer_sequence<T, Head, Tail...>>
  : sequence_element<I - 1, integer_sequence<T, Tail...>>
{ };

} // namespace stapl

#endif // STAPL_UTILITY_INTEGER_SEQUENCE_HPP

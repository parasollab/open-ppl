/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_FIND_FIRST_INDEX_HPP
#define STAPL_UTILITY_TUPLE_FIND_FIRST_INDEX_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/utility/integer_sequence.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to find the first item in a given tuple
/// that satisfies a given predicate.
///
/// @tparam Items      the tuple of input elements
/// @tparam Predicate  the predicate to be tested on each element
/// @tparam index      the starting index to start the search
/// @tparam items_left determines the number of items left to
///                    check the predicate
/// @tparam Arg        additional arguments to pass to Predicate after
///                    each element
//////////////////////////////////////////////////////////////////////
template <typename Items, template <typename...> class Predicate,
          int index,
          int items_left,
          typename... Arg>
struct find_first_index_impl
  : std::conditional<
      Predicate<typename tuple_element<index, Items>::type, Arg...>::value,
      std::integral_constant<int, index>,
      find_first_index_impl<Items, Predicate, index+1, items_left - 1, Arg...>
    >::type
{ };


//////////////////////////////////////////////////////////////////////
/// @brief The base case for @c find_first_index_impl
///
/// @tparam Items     the tuple of input elements
/// @tparam Predicate the predicate to be tested on each element
/// @tparam index     the starting index to start the search
/// @tparam Args      additional arguments to pass to Predicate after
///                   each element
//////////////////////////////////////////////////////////////////////
template<typename Items, template <typename...> class Predicate,
         int index, typename... Arg>
struct find_first_index_impl<Items, Predicate, index, 0, Arg...>
  : public std::integral_constant<int, -1>
{ };

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Finds the index of the first item in a given tuple that
/// satisfies a given predicate.
///
/// @tparam Items      the tuple of input elements
/// @tparam Predicate  the predicate to be tested on each element
/// @tparam Arg        additional arguments to pass to Predicate after
///                    each element
//////////////////////////////////////////////////////////////////////
template <typename Items,
          template <typename...> class Predicate,
          typename... Arg>
using find_first_index = detail::find_first_index_impl<
                           Items, Predicate,
                           0, tuple_size<Items>::value, Arg...>;

namespace detail {

template<typename Items, typename For,
          template <typename...> class Predicate,
          typename Indices, typename... Arg>
struct find_first_indices_impl;

template<typename Items, typename For,
          template <typename...> class Pred,
          size_t... Is, typename... Arg>
struct find_first_indices_impl<Items, For, Pred, index_sequence<Is...>, Arg...>
{
  using type =
    index_sequence<size_t{find_first_index<Items, Pred,
      typename tuple_element<Is, For>::type, Arg...>::value}...>;
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Return an index_sequence with the result of find_first_index
/// for all of the elements in For.
///
/// @tparam Items      the tuple of input elements
/// @tparam For        the tuple of predicate arguments
/// @tparam Predicate  the predicate to be tested on each element
/// @tparam Arg        additional arguments to pass to Predicate after
///                    each element of Items and For
///
/// @see index_sequence
/// @see find_first_index
/// @see find_first_indices
//////////////////////////////////////////////////////////////////////
template <typename Items,
          typename For,
          template <typename...> class Predicate,
          typename... Arg>
using find_first_indices_seq = detail::find_first_indices_impl<
                             Items, For, Predicate,
                             make_index_sequence<tuple_size<For>::value>,
                             Arg...>;

///////////////////////////////////////////////
/// @brief Metafunction to find the indices of values in one tuple in another
///   tuple.
///
/// For example, say we have the tuple <2,5,7> and the superset tuple
/// <1,2,3,5,7,8>. The output is expected to be <1,3,4>.
///
/// @tparam Subset Tuple of values that we are searching for.
/// @tparam AllValues Tuple of all values. Should be a superset of Subset
//////////////////////////////////////////////////////////////////////
template<typename Subset, typename AllValues,
         typename = make_index_sequence<tuple_size<Subset>::value>>
struct find_first_indices;

template<typename Subset, typename AllValues,
         std::size_t ...SubsetIndices>
struct find_first_indices<
  Subset, AllValues, index_sequence<SubsetIndices...>>
{
  using type = tuple<std::integral_constant<
                 std::size_t,
                 find_first_index<
                   AllValues,
                   std::is_same,
                   typename tuple_element<SubsetIndices, Subset>::type
                 >::value
               >...>;
};

} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_FIND_FIRST_INDEX_HPP

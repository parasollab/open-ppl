/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_TO_INDEX_HPP
#define STAPL_UTILITY_TUPLE_TO_INDEX_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>

namespace stapl {
namespace tuple_ops {

//////////////////////////////////////////////////////////////////////
/// @brief Creates an @ref index_sequence from a tuple of
/// @ref std::integral_constant.
///
/// @tparam Tuple of integral constants
//////////////////////////////////////////////////////////////////////
template<typename Tuple,
         typename = make_index_sequence<tuple_size<Tuple>::value>>
struct to_index_sequence;

template<typename Tuple, std::size_t... Indices>
struct to_index_sequence<Tuple, index_sequence<Indices...>>
{
  using type = index_sequence<tuple_element<Indices, Tuple>::type::value...>;
};

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_TO_INDEX_HPP

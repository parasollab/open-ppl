/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_FROM_INDEX_HPP
#define STAPL_UTILITY_TUPLE_FROM_INDEX_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>

namespace stapl {
namespace tuple_ops {

//////////////////////////////////////////////////////////////////////
/// @brief Creates a tuple of @ref std::integral_constant from a given
/// @ref index_sequence
///
/// @tparam IndexTuple an @ref index_sequence
///
/// @todo should be replaced by index_sequence
//////////////////////////////////////////////////////////////////////
template<typename IndexTuple>
struct from_index_sequence;

template<std::size_t... Indices>
struct from_index_sequence<index_sequence<Indices...>>
{
  using type = tuple<std::integral_constant<std::size_t, Indices>...>;
};

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_FROM_INDEX_HPP

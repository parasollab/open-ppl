/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_REVERSE_HPP
#define STAPL_UTILITY_TUPLE_REVERSE_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/utility/integer_sequence.hpp>

namespace stapl {

namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Reverses a given tuple. For example, tuple<char, int, double>
/// would become tuple<double, char, int>.
///
/// @tparam Tuple   a tuple to be reversed
/// @tparam IdxList an @c index_sequence to traverse the tuple
//////////////////////////////////////////////////////////////////////
template<
  typename Tuple,
  typename IdxList = make_index_sequence<tuple_size<Tuple>::value>>
struct reverse;


template<typename ...Elements, std::size_t... Indices>
struct reverse<tuple<Elements...>, index_sequence<Indices...>>
{
private:
  static constexpr size_t size = sizeof...(Indices);

public:
  using type = tuple<
                 typename tuple_element<
                   size - Indices - 1,
                   tuple<Elements...>>::type...>;
};


template<>
struct reverse<tuple<>, index_sequence<>>
{
  typedef tuple<> type;
};

} // namespace result_of

} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_REVERSE_HPP

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_POP_BACK_HPP
#define STAPL_UTILITY_TUPLE_POP_BACK_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/utility/utility.hpp>

namespace stapl {
namespace tuple_ops {

namespace result_of {

template <typename Tuple,
          typename IdxList = make_index_sequence<tuple_size<Tuple>::value - 1>>
struct pop_back;


template<typename ...Elements, std::size_t... Indices>
struct pop_back<tuple<Elements...>, index_sequence<Indices...>>
{
  using type = tuple<
                 typename tuple_element<
                   Indices, tuple<Elements...>>::type...>;

  static type call(tuple<Elements...> const& elements)
  {
    return type(std::get<Indices>(elements)...);
  }
};


} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief Returns a new tuple, with the last element of the original
/// removed.
///
/// @param t a tuple
//////////////////////////////////////////////////////////////////////
template<typename Tuple>
auto
pop_back(Tuple const& t)
STAPL_AUTO_RETURN((
  result_of::pop_back<Tuple>::call(t)
))

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_POP_BACK_HPP

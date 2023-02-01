/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_POP_FRONT_HPP
#define STAPL_UTILITY_POP_FRONT_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/utility.hpp>

namespace stapl {
namespace tuple_ops {

namespace result_of {

template<typename Tuple>
struct pop_front;

template<typename Head, typename ...Tail>
struct pop_front<tuple<Head, Tail...>>
{
  using type = tuple<Tail...>;

private:
  template<std::size_t... Indices>
  static type apply(tuple<Head, Tail...> const& elements,
      index_sequence<Indices...>)
  {
    return type(get<Indices+1>(elements)...);
  }

public:
  static type call(tuple<Head, Tail...> const& elements)
  {
    return apply(elements, make_index_sequence<sizeof...(Tail)>{});
  }
};

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief Returns a new tuple, with the first element of the original
/// removed.
///
/// @param t a tuple
//////////////////////////////////////////////////////////////////////
template<typename Tuple>
auto
pop_front(Tuple const& t)
STAPL_AUTO_RETURN((
  result_of::pop_front<Tuple>::call(t)
))

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_POP_FRONT_HPP

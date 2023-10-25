/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_PUSH_FRONT_HPP
#define STAPL_UTILITY_TUPLE_PUSH_FRONT_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/utility/utility.hpp>

namespace stapl {
namespace tuple_ops {

namespace result_of {

template<typename Tuple, typename T>
struct push_front;

template <typename T>
struct push_front<tuple<>, T>
{
  using type = tuple<T>;

  static type call(tuple<> const&, T const& val)
  {
    return type(val);
  }
};

template <typename ...Elements, typename T>
struct push_front<tuple<Elements...>, T>
{
  using type = tuple<T, Elements...>;

private:
  template<size_t... Indices>
  static type apply(tuple<Elements...> const& elements, T const& val,
      index_sequence<Indices...>)
  {
    return type(val, get<Indices>(elements)...);
  }

public:
  static type call(tuple<Elements...> const& elements, T const& val)
  {
    return apply(elements, val, make_index_sequence<sizeof...(Elements)>{});
  }
};


} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief Returns a new tuple with @c val added at the beginning.
///
/// @param t   a tuple
/// @param val the new element to be added at the beginning
//////////////////////////////////////////////////////////////////////
template<typename ...Elements, typename T>
auto push_front(tuple<Elements...> const& t, T const& val)
STAPL_AUTO_RETURN((
  result_of::push_front<tuple<Elements...>, T>::call(t, val)
))

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_PUSH_FRONT_HPP

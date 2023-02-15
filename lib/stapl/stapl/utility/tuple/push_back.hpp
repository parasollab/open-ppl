/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_PUSH_BACK_HPP
#define STAPL_UTILITY_TUPLE_PUSH_BACK_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/utility.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>

namespace stapl {
namespace tuple_ops {

namespace result_of {

template<typename Tuple, typename T>
struct push_back;

template <typename T>
struct push_back<tuple<>, T>
{
  using type = tuple<T>;

  static type call(tuple<> const&, T const& val)
  {
    return type(val);
  }
};

template <typename ...Elements, typename T>
struct push_back<tuple<Elements...>, T>
{
  using type = tuple<Elements..., T>;

private:
  template<size_t... Indices>
  static type apply(tuple<Elements...> const& elements, T const& val,
      index_sequence<Indices...>)
  {
    return type(get<Indices>(elements)..., val);
  }

public:
  static type call(tuple<Elements...> const& elements, T const& val)
  {
    return apply(elements, val, make_index_sequence<sizeof...(Elements)>{});
  }
};

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief Returns a new tuple with @c val added at the end.
///
/// @param t   a tuple
/// @param val the new element to be added at the end
//////////////////////////////////////////////////////////////////////
template<typename ...Elements, typename T>
auto push_back(tuple<Elements...> const& t, T const& val)
STAPL_AUTO_RETURN((
  result_of::push_back<tuple<Elements...>, T>::call(t, val)
))

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_PUSH_BACK_HPP

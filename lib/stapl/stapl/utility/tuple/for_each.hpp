/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_FOR_EACH_HPP
#define STAPL_UTILITY_TUPLE_FOR_EACH_HPP

#include <stapl/utility/tuple/tuple.hpp>

namespace stapl {
namespace tuple_ops {

namespace detail {

template<std::size_t Num, typename Functor, typename ...Elements>
struct for_each_impl
{
  void operator()(tuple<Elements...> const& elements,
                  Functor const& f) const
  {
    for_each_impl<Num - 1, Functor, Elements...>()(elements, f);

    f(std::get<Num - 1>(elements));
  }

  void operator()(tuple<Elements...>& elements,
                  Functor const& f) const
  {
    for_each_impl<Num - 1, Functor, Elements...>()(elements, f);

    f(std::get<Num - 1>(elements));
  }

};


template<typename Functor, typename ...Elements>
struct for_each_impl<0, Functor, Elements...>
{
  void operator()(tuple<Elements...> const&, Functor const&) const
  { }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Applies a function to every element of a given tuple in order.
///
/// @param elements a tuple
/// @param f        a unary operator
//////////////////////////////////////////////////////////////////////
template<typename Functor, typename ...Elements>
void for_each(tuple<Elements...> const& elements, Functor const& f)
{
  const size_t size = sizeof...(Elements);

  detail::for_each_impl<size, Functor, Elements...>()(elements, f);
}


//////////////////////////////////////////////////////////////////////
/// @copybrief for_each
///
/// @param elements a tuple
/// @param f        a unary operator
//////////////////////////////////////////////////////////////////////
template<typename Functor, typename ...Elements>
void for_each(tuple<Elements...>& elements, Functor const& f)
{
  const size_t size = sizeof...(Elements);

  detail::for_each_impl<size, Functor, Elements...>()(elements, f);
}

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_FOR_EACH_HPP

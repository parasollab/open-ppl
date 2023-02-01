/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_FRONT_HPP
#define STAPL_UTILITY_TUPLE_FRONT_HPP

#include <type_traits>
#include <stapl/utility/utility.hpp>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>

namespace stapl {
namespace tuple_ops {

namespace result_of {

template<typename Tuple>
struct front
{
  using type = typename stapl::tuple_element<0, Tuple>::type;
};

template<typename... Elements>
struct front<tuple<Elements...>&>
{
  using type = typename std::add_lvalue_reference<
                 typename stapl::tuple_element<
                   0, tuple<Elements...>>::type
               >::type;
};

template <typename... Elements>
struct front<const tuple<Elements...>&>
{
  using type = typename std::add_const<
                 typename std::add_lvalue_reference<
                   typename stapl::tuple_element<
                     0, tuple<Elements...>>::type
                 >::type
               >::type;
};

template <typename... Elements>
struct front<tuple<Elements...>&&>
{
  using type = typename std::add_rvalue_reference<
                 typename stapl::tuple_element<
                   0, tuple<Elements...>>::type
               >::type;
};

} // namespace result_of

//////////////////////////////////////////////////////////////////////
/// @brief Returns the first element of a given tuple.
///
/// @param t a tuple
//////////////////////////////////////////////////////////////////////
template<typename Tuple>
auto front(Tuple&& t)
STAPL_AUTO_RETURN((
  std::get<0>(std::forward<Tuple>(t))
))

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_FRONT_HPP

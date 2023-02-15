/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_BACK_HPP
#define STAPL_UTILITY_TUPLE_BACK_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/utility/utility.hpp>

namespace stapl {
namespace tuple_ops {

namespace result_of {


template<typename Tuple>
struct back
{
  using type = typename stapl::tuple_element<
                 stapl::tuple_size<Tuple>::value - 1,
                 Tuple>::type;
};

template <typename... Elements>
struct back<tuple<Elements...>&>
{
  using type = typename std::add_lvalue_reference<
                 typename stapl::tuple_element<
                   sizeof...(Elements) - 1,
                   tuple<Elements...>>::type
               >::type;
};

template <typename... Elements>
struct back<const tuple<Elements...>&>
{
  using type = typename std::add_const<
                 typename std::add_lvalue_reference<
                   typename stapl::tuple_element<
                     sizeof...(Elements) - 1,
                     tuple<Elements...>>::type
                 >::type
               >::type;
};

template <typename... Elements>
struct back<tuple<Elements...>&&>
{
  using type = typename std::add_rvalue_reference<
                 typename stapl::tuple_element<
                   sizeof...(Elements) - 1,
                   tuple<Elements...>>::type
               >::type;
};


} // namespace result_of

template<typename Tuple>
auto back(Tuple&& t)
STAPL_AUTO_RETURN((
  std::get<stapl::tuple_size<typename std::decay<Tuple>::type>::value - 1>(
    std::forward<Tuple>(t))
))


} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_BACK_HPP

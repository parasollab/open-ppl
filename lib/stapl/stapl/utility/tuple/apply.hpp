/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_APPLY_HPP
#define STAPL_UTILITY_TUPLE_APPLY_HPP

#include <stapl/runtime/type_traits/callable_traits.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include "tuple_size.hpp"
#include <type_traits>
#include <utility>
#include <stapl/utility/utility.hpp>

namespace stapl {
namespace tuple_ops {

//////////////////////////////////////////////////////////////////////
/// @brief Calls @p f with arguments unpacked from tuple @p t in the order given
///        by the @ref index_sequence.
//////////////////////////////////////////////////////////////////////
template<typename F, typename Tuple, std::size_t... Is>
constexpr auto
apply_impl(F&& f, Tuple&& t, index_sequence<Is...>)
STAPL_AUTO_RETURN(
  std::forward<F>(f)(get<Is>(std::forward<Tuple>(t))...)
)

//////////////////////////////////////////////////////////////////////
/// @brief Invokes pointer member function @p pmf on object @p obj with
///        arguments unpacked from tuple @p t in the order given by the
///        @ref index_sequence.
//////////////////////////////////////////////////////////////////////
template<typename Obj, typename PMF, typename Tuple, std::size_t... Is>
constexpr typename callable_traits<PMF>::result_type
apply_impl(Obj&& obj, PMF const& pmf, Tuple&& t, index_sequence<Is...>)
{
  return (std::forward<Obj>(obj).*pmf)(get<Is>(std::forward<Tuple>(t))...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Calls @p f with arguments unpacked from tuple @p t.
//////////////////////////////////////////////////////////////////////
template<typename F, typename Tuple>
constexpr auto apply(F&& f, Tuple&& t)
STAPL_AUTO_RETURN(
  apply_impl(
    std::forward<F>(f),
    std::forward<Tuple>(t),
    make_index_sequence<tuple_size<typename std::decay<Tuple>::type>::value>{})
)


//////////////////////////////////////////////////////////////////////
/// @brief Invokes pointer member function @p pmf on object @p obj with
///        arguments unpacked from tuple @p t.
//////////////////////////////////////////////////////////////////////
template<typename Obj, typename PMF, typename Tuple>
constexpr typename callable_traits<PMF>::result_type
apply(Obj&& obj, PMF const& pmf, Tuple&& t)
{
  return apply_impl(
           std::forward<Obj>(obj),
           pmf,
           std::forward<Tuple>(t),
           make_index_sequence<
             tuple_size<typename std::decay<Tuple>::type>::value
           >{});
}

} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_APPLY_HPP

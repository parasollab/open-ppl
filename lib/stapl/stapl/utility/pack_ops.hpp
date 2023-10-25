/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_PACK_OPS_HPP
#define STAPL_UTILITY_PACK_OPS_HPP

#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/utility.hpp>

namespace stapl {

namespace pack_ops {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction that performs a logical and of its compile-time
///        boolean template parameters
//////////////////////////////////////////////////////////////////////
template<bool Head, bool... Tail>
struct and_
  : public std::integral_constant<bool, Head & and_<Tail...>::value>
{ };

template<bool Head>
struct and_<Head>
  : public std::integral_constant<bool, Head>
{ };


namespace functional {

//////////////////////////////////////////////////////////////////////
/// @brief Performs logical and on a variadic pack of elements.
//////////////////////////////////////////////////////////////////////
template<typename B>
constexpr bool and_(B&& b)
{
  return b;
}

template<typename B0, typename B1, typename... Bs>
constexpr bool and_(B0&& b0, B1&& b1, Bs&&... bs)
{
  return and_(std::forward<B0>(b0) &&
    and_(std::forward<B1>(b1), std::forward<Bs>(bs)...));
}


//////////////////////////////////////////////////////////////////////
/// @brief Performs logical or on a variadic pack of elements.
//////////////////////////////////////////////////////////////////////
template<typename B>
constexpr bool or_(B&& b)
{
  return b;
}


template<typename B0, typename B1, typename... Bs>
constexpr bool or_(B0&& b0, B1&& b1, Bs&&... bs)
{
  return or_(std::forward<B0>(b0) ||
    or_(std::forward<B1>(b1), std::forward<Bs>(bs)...));
}


//////////////////////////////////////////////////////////////////////
/// @brief Multiplies a variadic pack of elements.
//////////////////////////////////////////////////////////////////////
template<typename B>
constexpr typename std::decay<B>::type
multiplies_(B&& b)
{
  return b;
}


template<typename B0, typename B1, typename... Bs>
constexpr typename std::decay<B0>::type
multiplies_(B0&& b0, B1&& b1, Bs&&... bs)
{
  return multiplies_(std::forward<B0>(b0) *
    multiplies_(std::forward<B1>(b1), std::forward<Bs>(bs)...));
}


//////////////////////////////////////////////////////////////////////
/// @brief Adds a variadic pack of elements.
//////////////////////////////////////////////////////////////////////
template<typename B>
constexpr typename std::decay<B>::type
plus_(B&& b)
{
  return b;
}


template<typename B0, typename B1, typename... Bs>
constexpr typename std::decay<B0>::type
plus_(B0&& b0, B1&& b1, Bs&&... bs)
{
  return plus_(std::forward<B0>(b0) +
    plus_(std::forward<B1>(b1), std::forward<Bs>(bs)...));
}


//////////////////////////////////////////////////////////////////////
/// @brief Applies functor to each argument in a variadic pack.
//////////////////////////////////////////////////////////////////////
template<typename F>
void for_each_(F&& f)
{ }


template<typename F, typename Arg0, typename... Args>
void for_each_(F&& f, Arg0&& arg0, Args&&... args)
{
  f(std::forward<Arg0>(arg0));
  for_each_(std::forward<F>(f), std::forward<Args>(args)...);
}

} // namespace functional


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute the type of the <tt>I</tt>-th parameter
///        in the parameter pack @c Ts...
//////////////////////////////////////////////////////////////////////
template<int I, typename... Ts>
struct pack_element;

template<typename T, typename... Ts>
struct pack_element<0, T, Ts...>
{
  using type = T;
};

template<int I, typename T, typename... Ts>
struct pack_element<I, T, Ts...>
{
  using type = typename pack_element<I - 1, Ts...>::type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Returns the <tt>I</tt>-th argument of the argument list @c args...
//////////////////////////////////////////////////////////////////////
template<int I, typename T, typename... Ts>
constexpr auto get(T&& t, Ts&&... /*args*/) ->
  typename std::enable_if<(I == 0), decltype(std::forward<T>(t))>::type
{
  return std::forward<T>(t);
}

template<int I, typename T, typename... Ts>
constexpr auto get(T&& /*t*/, Ts&&... args) ->
  typename std::enable_if<(I > 0), decltype(
    std::forward<typename pack_element<I, T, Ts...>::type>(
      std::declval<typename pack_element<I, T, Ts...>::type>() )
  )>::type
{
  using result_type = typename pack_element<I, T, Ts...>::type;
  return
    std::forward<result_type>( get<I - 1>(std::forward<Ts>(args)...) );
}

//////////////////////////////////////////////////////////////////////
/// @brief Calls the provided functor @c func with arguments @c args...,
///        reordered by the permutation sequence @c is.
/// @sa sliced_mf::apply_get_impl()
//////////////////////////////////////////////////////////////////////
template<typename F, std::size_t... Is, typename... Ts>
constexpr auto
call_with_permuted_args(F&& func, index_sequence<Is...> const& is, Ts&&... args)
STAPL_AUTO_RETURN (
  std::forward<F>(func)(get<Is>(std::forward<Ts>(args)...)...)
)

} // namespace pack_ops

} // namespace stapl

#endif // STAPL_UTILITY_PACK_OPS_HPP

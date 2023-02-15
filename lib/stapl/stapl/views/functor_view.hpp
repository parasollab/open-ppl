/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_FUNCTOR_VIEW_HPP
#define STAPL_VIEWS_FUNCTOR_VIEW_HPP

#include <stapl/views/array_view.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/containers/generators/functor.hpp>

#include <iostream>

namespace stapl {


//////////////////////////////////////////////////////////////////////
/// @brief Defines a nested trait type that is the type of a functor view
/// parameterized with a functor type
//////////////////////////////////////////////////////////////////////
template <typename FunctorContainer>
struct functor_view_type
{
  using type = multiarray_view<FunctorContainer>;
};

//////////////////////////////////////////////////////////////////////
/// @brief Defines a nested trait type that is the type of a functor view
/// parameterized with a functor type
//////////////////////////////////////////////////////////////////////
template <typename Func, typename... Distribution>
struct functor_view_type<functor_container<Func, 1, Distribution...>>
{
  using type = array_view<functor_container<Func, 1, Distribution...>>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper function that creates an array view on top of a
///        functor container.
/// @param size Number of elements in the container.
/// @param func Functor to be called on each [] operator call.
//////////////////////////////////////////////////////////////////////
template <
  typename Size, typename Func, typename... Distribution,
  typename = typename std::enable_if<!std::is_scalar<Size>::value, void>::type>
typename functor_view_type<
  functor_container<Func, tuple_size<Size>::value, Distribution...>>::type
functor_view(Size const& size, Func const& func)
{
  using functor_cont_t =
    functor_container<Func, tuple_size<Size>::value, Distribution...>;
  return typename functor_view_type<functor_cont_t>::type(
    new functor_cont_t(size, func));
}

//////////////////////////////////////////////////////////////////////
/// @brief Helper function that creates an array view on top of a
///        functor container.
/// @param size Number of elements in the container.
/// @param func Functor to be called on each [] operator call.
//////////////////////////////////////////////////////////////////////
template <
  typename Size, typename Func, typename... Distribution,
  typename = typename std::enable_if<std::is_scalar<Size>::value, void>::type>
typename functor_view_type<functor_container<Func, 1, Distribution...>>::type
functor_view(Size const& size, Func const& func)
{
  using functor_cont_t = functor_container<Func, 1, Distribution...>;
  return typename functor_view_type<functor_cont_t>::type(
    new functor_cont_t(size, func));
}

} // stapl namespace

#endif // STAPL_VIEWS_FUNCTOR_VIEW_HPP

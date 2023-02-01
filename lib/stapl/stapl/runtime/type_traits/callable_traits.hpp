/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TYPE_TRAITS_CALLABLE_TRAITS_HPP
#define STAPL_RUNTIME_TYPE_TRAITS_CALLABLE_TRAITS_HPP

#include <tuple>

namespace stapl {

////////////////////////////////////////////////////////////////////
/// @brief Callable introspection class.
///
/// This class provides member typedef @c object_type for the target object
/// type, member typedef @c result_type for the return type and member
/// typedef @c parameter_types that is an @c std::tuple with the parameter types
/// of the function.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename F>
struct callable_traits
{
  using result_type =
    typename callable_traits<decltype(&F::operator())>::result_type;
  using parameter_types =
    typename callable_traits<decltype(&F::operator())>::parameter_types;
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref callable_traits for pointers to member
///        functions.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename R,
         typename C,
         typename... T>
struct callable_traits<R(C::*)(T...)>
{
  using object_type     = C;
  using result_type     = R;
  using parameter_types = std::tuple<T...>;
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref callable_traits for pointers to @c const
///        member functions.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename R,
         typename C,
         typename... T>
struct callable_traits<R(C::*)(T...) const>
{
  using object_type     = C;
  using result_type     = R;
  using parameter_types = std::tuple<T...>;
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref callable_traits for pointers to @c volatile
///        member functions.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename R,
         typename C,
         typename... T>
struct callable_traits<R(C::*)(T...) volatile>
{
  using object_type     = C;
  using result_type     = R;
  using parameter_types = std::tuple<T...>;
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref callable_traits for pointers to @c const
///        @c volatile member functions.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename R,
         typename C,
         typename... T>
struct callable_traits<R(C::*)(T...) const volatile>
{
  using object_type     = C;
  using result_type     = R;
  using parameter_types = std::tuple<T...>;
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref callable_traits for @c const pointers.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename R,
         typename C>
struct callable_traits<R(C::*const)>
: public callable_traits<R(C::*)>
{ };


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref callable_traits for functions.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename R,
         typename... T>
struct callable_traits<R(T...)>
{
  using result_type     = R;
  using parameter_types = std::tuple<T...>;
};


////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref callable_traits for pointers to functions.
///
/// @ingroup ARMITypeTraits
////////////////////////////////////////////////////////////////////
template<typename R,
         typename... T>
struct callable_traits<R(*)(T...)>
: public callable_traits<R(T...)>
{ };

} // namespace stapl

#endif

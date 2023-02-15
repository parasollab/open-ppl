/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_REQUEST_ARGUMENTS_HPP
#define STAPL_RUNTIME_REQUEST_ARGUMENTS_HPP

#include "arg_storage.hpp"
#include <stapl/utility/integer_sequence.hpp>
#include <algorithm>
#include <numeric>
#include <initializer_list>
#include <tuple>
#include <utility>

namespace stapl {

namespace runtime {

template<typename...>
struct make_arguments;

//////////////////////////////////////////////////////////////////////
/// @brief Metafuction for creating a tuple of @ref arg_storage_t to store
///        the objects in list @p T.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename F, typename... T>
struct make_arguments<F, T...>
{
  using type = typename make_arguments<decltype(&F::operator()), T...>::type;
};

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Metafuction for creating a tuple of @ref arg_storage_t to store
///        the objects in list @p T.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename R, typename C, typename... ArgTypes, typename... T>
struct make_arguments<R (C::*)(ArgTypes...), T...>
{
  using type = std::tuple<arg_storage_t<T, ArgTypes>...>;
};

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arguments for const member functions.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename R, typename C, typename... ArgTypes, typename... T>
struct make_arguments<R (C::*)(ArgTypes...) const, T...>
{
  using type = std::tuple<arg_storage_t<T, ArgTypes>...>;
};

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arguments for volatile member functions.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename R, typename C, typename... ArgTypes, typename... T>
struct make_arguments<R (C::*)(ArgTypes...) volatile, T...>
{
  using type = std::tuple<arg_storage_t<T, ArgTypes>...>;
};

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arguments for const volatile member
///        functions.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename R, typename C, typename... ArgTypes, typename... T>
struct make_arguments<R (C::*)(ArgTypes...) const volatile, T...>
{
  using type = std::tuple<arg_storage_t<T, ArgTypes>...>;
};

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arguments for const pointers.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename R, typename C, typename... T>
struct make_arguments<R (C::* const), T...>
: public make_arguments<R (C::*), T...>
{ };

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arguments for functions.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename R, typename... ArgTypes, typename... T>
struct make_arguments<R(ArgTypes...), T...>
{
  using type = std::tuple<arg_storage_t<T, ArgTypes>...>;
};

//////////////////////////////////////////////////////////////////////
/// @internal
/// @brief Specialization of @ref make_arguments for function pointers.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename R, typename... ArgTypes, typename... T>
struct make_arguments<R(*)(ArgTypes...), T...>
{
  using type = std::tuple<arg_storage_t<T, ArgTypes>...>;
};


//////////////////////////////////////////////////////////////////////
/// @brief A tuple of @ref arg_storage_t to store the objects in list @p T.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename F, typename... T>
using arguments_t = typename make_arguments<F, T...>::type;


//////////////////////////////////////////////////////////////////////
/// @brief Returns the dynamic size required for storing the objects @p t... in
///        a @ref arguments_t.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Tuple, std::size_t... I, typename... T>
std::size_t dynamic_size(index_sequence<I...>, T&&... t) noexcept
{
  const std::initializer_list<std::size_t> a =
    { std::tuple_element<I, Tuple>::type::packed_size(std::forward<T>(t))... };

  return std::accumulate(a.begin(), a.end(), std::size_t(0));
}


//////////////////////////////////////////////////////////////////////
/// @brief Calls @p f with the stored objects in @p args passed as arguments.
///
/// @param base Base address that @p args are stored in.
/// @param f    Function to invoke.
/// @param args Arguments to unpack.
///
/// @return The result of @p f.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename F, typename Tuple, std::size_t... I>
auto invoke(F&& f, Tuple& args, void* const base, index_sequence<I...>)
  -> decltype(std::forward<F>(f)(std::get<I>(args).get(base)...))
{
  return std::forward<F>(f)(std::get<I>(args).get(base)...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Calls the member function @p pmf on @p ref with the stored objects in
///        @p args passed as arguments.
///
/// @param pmf  Member function to invoke.
/// @param ref  Object to invoke @p pmf on.
/// @param args Arguments to unpack.
/// @param base Base address that @p args are stored in.
///
/// @return The result of @p pmf.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename MemFun,
         typename Base,
         typename Tuple,
         std::size_t... I>
auto invoke(MemFun const& pmf, Base& ref, Tuple& args,
            void* const base, index_sequence<I...>)
  -> decltype((ref.*pmf)(std::get<I>(args).get(base)...))
{
  return (ref.*pmf)(std::get<I>(args).get(base)...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Calls the member function @p pmf on @p ref with the stored objects in
///        @p args passed as arguments.
///
/// @param pmf  Member function to invoke.
/// @param ref  Object to invoke @p pmf on.
/// @param args Arguments to unpack.
/// @param base Base address that @p args are stored in.
/// @param size Variable to store how many bytes were unpacked.
///
/// @return The result of @p pmf.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename MemFun,
         typename Base,
         typename Tuple,
         std::size_t... I>
auto invoke(MemFun const& pmf, Base& ref, Tuple& args,
            void* const base, std::size_t& size, index_sequence<I...>)
  -> decltype((ref.*pmf)(std::get<I>(args).get(base, size)...))
{
  return (ref.*pmf)(std::get<I>(args).get(base, size)...);
}


//////////////////////////////////////////////////////////////////////
/// @brief Releases any resources associated with the stored objects if
///        @ref invoke() was not called.
///
/// @todo Not implemented.
///
/// @ingroup requestBuildingBlock
//////////////////////////////////////////////////////////////////////
template<typename Tuple, std::size_t... I>
void cleanup(Tuple& /*args*/, index_sequence<I...>)
{ }

} // namespace runtime

} // namespace stapl

#endif

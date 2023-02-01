/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTIILTY_TUPLE_HOMOGENEOUS_TUPLE_HPP
#define STAPL_UTIILTY_TUPLE_HOMOGENEOUS_TUPLE_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple/ignore_index.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Trivial type metafunction reflecting second type parameter
///   Used in variadic expansion of integral parameter packs of a
///   class that wishes to define a tuple of homogeneous types
///   (i.e., @ref indexed_domain).
//////////////////////////////////////////////////////////////////////
template<int N, typename T>
struct tuple_type_reflector
{
  typedef T type;
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Type metafunction return tuple type of length @p N with
///  with all elements of type @p T.
//////////////////////////////////////////////////////////////////////
template <int N, typename T,
          typename IdxList = make_index_sequence<N>>
struct homogeneous_tuple_type;


template <int N, typename T, std::size_t... Indices>
struct homogeneous_tuple_type<N, T, index_sequence<Indices...>>
{
  using type = tuple<
                 typename detail::tuple_type_reflector<Indices, T>::type...>;
};

template<int N, typename T>
using homogeneous_tuple_type_t = typename homogeneous_tuple_type<N,T>::type;

namespace detail {

template<int N, typename T,
         typename Indices = make_index_sequence<N>>
struct homogeneous_tuple;

template<int N, typename T, std::size_t... Indices>
struct homogeneous_tuple<N, T, index_sequence<Indices...>>
{
  using type = typename homogeneous_tuple_type<N, T>::type;

  static type apply(T const& u)
  {
    return type(ignore_index<Indices>(u)...);
  }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to create a tuple of homogeneous types where
///        each element of the tuple has the same value.
///
/// @tparam Number of elements in the tuple
/// @param t Value that is going to be copied to all elements in the tuple
//////////////////////////////////////////////////////////////////////
template<int N, typename T>
typename homogeneous_tuple_type<N, T>::type
homogeneous_tuple(T const& t)
{
  return detail::homogeneous_tuple<N, typename std::decay<T>::type>::apply(t);
}

template<typename T>
struct make_homogeneous_tuple;

template<typename... T>
struct make_homogeneous_tuple<tuple<T...> >
{
  template<typename... Args>
  static tuple<T...> apply(Args&&... args)
  {
    return std::make_tuple(T(args...)...);
  }
};

namespace detail {

template<int I, typename Tuple, typename Array>
struct homogeneous_tuple_to_array_impl
{
  static void apply(Tuple const& t, Array& a)
  {
    a[I] = std::get<I>(t);
    homogeneous_tuple_to_array_impl<I-1, Tuple, Array>::apply(t, a);
  }
};

template<typename T, typename A>
struct homogeneous_tuple_to_array_impl<-1, T, A>
{
  static void apply(T const&, A&)
  { }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Convert a tuple of homogeneous types to an std::array
//////////////////////////////////////////////////////////////////////
template<typename Tuple>
struct homogeneous_tuple_to_array
{
  using type = std::array<
                 typename tuple_element<0, Tuple>::type,
                 tuple_size<Tuple>::value>;

  static type apply(Tuple const& t)
  {
    using impl_t = detail::homogeneous_tuple_to_array_impl<
                     tuple_size<Tuple>::value-1, Tuple, type>;
    type a;
    impl_t::apply(t, a);
    return a;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Convert a tuple of homogeneous types to an std::array
///
/// @param t the input tuple to be converted to an array
//////////////////////////////////////////////////////////////////////
template <typename Tuple>
typename homogeneous_tuple_to_array<Tuple>::type
convert_homogeneous_tuple_to_array(Tuple const& t)
{
  return homogeneous_tuple_to_array<Tuple>::apply(t);
}

} // namespace stapl

#endif // STAPL_UTIILTY_TUPLE_HOMOGENEOUS_TUPLE_HPP

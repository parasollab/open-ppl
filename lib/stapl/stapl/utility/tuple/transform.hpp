/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_TRANSFORM_HPP
#define STAPL_UTILITY_TUPLE_TRANSFORM_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple/tuple_size.hpp>
#include <stapl/utility/tuple/tuple_element.hpp>
#include <boost/utility/result_of.hpp>
#include <stapl/utility/utility.hpp>

namespace stapl {

namespace result_of {

template <
  typename Tuple,
  template <typename> class Functor>
struct transform1;


template <typename ...Elements, template <typename> class Functor>
struct transform1<tuple<Elements...>, Functor>
{
  using type = tuple<typename Functor<Elements>::type...>;
};


template <
  typename Tuple1,
  typename Tuple2,
  template <typename, typename> class Functor>
struct transform2;


template <typename ...Elements1, typename ...Elements2,
         template <typename, typename> class Functor>
struct transform2<tuple<Elements1...>, tuple<Elements2...>, Functor>
{
  using type = tuple<typename Functor<Elements1, Elements2>::type...>;
};


template <
  typename Tuple1,
  typename Tuple2,
  typename Functor = void>
struct transform;


template <typename ...Elements1, typename ...Elements2, typename Functor>
struct transform<tuple<Elements1...>, tuple<Elements2...>, Functor>
{
  using type = tuple<
      typename boost::result_of<Functor(Elements1, Elements2)>::type...>;
};

template <typename ...Elements1, typename Functor>
struct transform<tuple<Elements1...>, Functor, void>
{
  using type = tuple<typename boost::result_of<Functor(Elements1)>::type...>;
};

} // namespace result_of

namespace tuple_ops {

namespace detail {

template <
  typename Functor,
  typename Tuple1,
  typename IdxList = make_index_sequence<tuple_size<Tuple1>::value>>
struct transform1_impl;


template <typename Functor,
          typename ...Elements1,
          std::size_t... Indices>
struct transform1_impl<Functor,
                       tuple<Elements1...>,
                       index_sequence<Indices...>>
{
  auto operator()(tuple<Elements1...> const& t1,
                  Functor f) const
  STAPL_AUTO_RETURN(
    std::make_tuple(f(std::get<Indices>(t1))...)
  )
};


template <
  typename Functor,
  typename Tuple1,
  typename Tuple2,
  typename IdxList = make_index_sequence<tuple_size<Tuple1>::value>>
struct transform2_impl;


template <typename Functor,
         typename ...Elements1,
         typename ...Elements2,
         std::size_t... Indices>
struct transform2_impl<Functor,
                       tuple<Elements1...>,
                       tuple<Elements2...>,
                       index_sequence<Indices...>
>
{
  auto operator()(tuple<Elements1...> const& t1,
                  tuple<Elements2...> const& t2,
                  Functor f) const
  STAPL_AUTO_RETURN(
    std::make_tuple(f(std::get<Indices>(t1), std::get<Indices>(t2))...)
  )
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Creates a new tuple by applying @c f on each element of the
/// given tuple.
///
/// @param t1 a tuple
/// @param f  a unary operator
//////////////////////////////////////////////////////////////////////
template <typename Tuple1, typename Functor>
inline
auto
transform(Tuple1 const& t1, Functor&& f)
STAPL_AUTO_RETURN((
  detail::transform1_impl<Functor, Tuple1>()(t1, std::forward<Functor>(f))
))


//////////////////////////////////////////////////////////////////////
/// @brief Creates a new tuple by applying @c f on each element of the
/// given tuples.
///
/// @param t1 a tuple
/// @param t2 a tuple
/// @param f  a binary operator
//////////////////////////////////////////////////////////////////////
template <typename Tuple1, typename Tuple2, typename Functor>
inline
auto
transform(Tuple1 const& t1, Tuple2 const& t2, Functor&& f)
STAPL_AUTO_RETURN((
  detail::transform2_impl<Functor, Tuple1, Tuple2>()(t1, t2,
    std::forward<Functor>(f))
))

} // namespace tuple_ops


} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_TRANSFORM_HPP

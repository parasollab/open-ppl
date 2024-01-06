/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <boost/mpl/sequence_tag.hpp>
#include <boost/mpl/pop_front_fwd.hpp>
#include <boost/mpl/push_front_fwd.hpp>
#include <boost/mpl/push_back_fwd.hpp>
#include <boost/mpl/front_fwd.hpp>
#include <boost/mpl/empty_fwd.hpp>
#include <boost/mpl/size_fwd.hpp>
#include <boost/mpl/at_fwd.hpp>
#include <boost/mpl/back_fwd.hpp>
#include <boost/mpl/clear_fwd.hpp>
#include <boost/mpl/pop_back_fwd.hpp>
#include <boost/mpl/iterator_tags.hpp>
#include <boost/mpl/next_prior.hpp>
#include <boost/mpl/deref.hpp>
#include <boost/mpl/begin_end_fwd.hpp>

//////////////////////////////////////////////////////////////////////
/// @file
/// Specializations of Boost.MPL internal class templates to
/// conform std::tuple to the mpl sequence concept (e.g., mpl::list)
/// Allows stapl::tuple in C++11 mode to use mpl metafunctions.
///
/// Derived from http://stackoverflow.com/a/15865204/429110 (no license).
//////////////////////////////////////////////////////////////////////

namespace boost {

namespace mpl {

namespace aux {

struct std_tuple;

} // namespace aux


template<typename ...Args>
struct sequence_tag<std::tuple<Args...> >
{
  typedef aux::std_tuple type;
};


template<>
struct front_impl<aux::std_tuple>
{
  template<typename Tuple>
  struct apply
    : std::tuple_element<0, Tuple>
  { };
};


template<>
struct empty_impl<aux::std_tuple>
{
  template<typename Tuple>
   struct apply
      : std::integral_constant<bool, std::tuple_size<Tuple>::value == 0>
  { };
};


template<>
struct pop_front_impl<aux::std_tuple>
{
  template<typename Tuple>
  struct apply;

  template<typename First, typename ...Types>
  struct apply<std::tuple<First, Types...> >
  {
    typedef std::tuple<Types...> type;
  };
};


template<>
struct push_front_impl<aux::std_tuple>
{
  template<typename Tuple, typename T>
  struct apply;

  template<typename T, typename ...Args>
  struct apply<std::tuple<Args...>, T>
  {
    typedef std::tuple<T, Args...> type;
  };
};


template<>
struct push_back_impl<aux::std_tuple>
{
  template<typename Tuple, typename T>
  struct apply;

  template<typename T, typename ...Args>
  struct apply<std::tuple<Args...>, T>
  {
    typedef std::tuple<Args..., T> type;
  };
};


template<>
struct size_impl<aux::std_tuple>
{
  template<typename Tuple>
  struct apply
    : public std::tuple_size<Tuple>
  { };
};


template<>
struct at_impl<aux::std_tuple>
{
  template<typename Tuple, typename N>
  struct apply
      : std::tuple_element<N::value, Tuple>
  { };
};


template<>
struct back_impl<aux::std_tuple>
{
  template<typename Tuple>
  struct apply
      : std::tuple_element<std::tuple_size<Tuple>::value - 1, Tuple>
  { };
};


template<>
struct clear_impl<aux::std_tuple>
{
  template<typename Tuple>
  struct apply
  {
    typedef std::tuple<> type;
  };
};


template<>
struct pop_back_impl<aux::std_tuple>
{
  template<int ...>
  struct tuple_seq
  { };

  template<int N, int ...S>
  struct tuple_gens
    : public tuple_gens<N-1, N-1, S...>
  { };

  template<int ...S>
  struct tuple_gens<0, S...>
  {
    typedef tuple_seq<S...> type;
  };

  template < typename Tuple, typename Index>
  struct apply_impl;

  template <typename Tuple, int ... S>
  struct apply_impl<Tuple, tuple_seq<S...>>
  {
    typedef std::tuple<typename std::tuple_element<S, Tuple>::type...> type;
  };

  template<typename Tuple>
  struct apply
    : apply_impl<
        Tuple, typename tuple_gens<std::tuple_size<Tuple>::value - 1>::type
      >
  { };
};


template<typename ...Args>
struct tuple_iter;


template<typename ...Args>
struct tuple_iter<std::tuple<Args...>>
{
  typedef aux::std_tuple       tag;
  typedef forward_iterator_tag category;
};


template<>
struct begin_impl<aux::std_tuple>
{
  template<typename Tuple>
  struct apply
  {
    typedef tuple_iter<Tuple> type;
  };
};


template<>
struct end_impl<aux::std_tuple>
{
  template<typename>
  struct apply
  {
    typedef tuple_iter<std::tuple<>> type;
  };
};


template<typename First, typename ... Args>
struct deref<tuple_iter<std::tuple<First, Args...>>>
{
  typedef First type;
};


template<typename First, typename ...Args>
struct next<tuple_iter<std::tuple<First, Args...>>>
{
  typedef tuple_iter<std::tuple<Args...>> type;
};

} // namespace mpl

} // namespace boost


/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_INSERT_SORTED_HPP
#define STAPL_UTILITY_TUPLE_INSERT_SORTED_HPP


#include "tuple.hpp"
#include <stapl/utility/type_less.hpp>
#include <stapl/utility/type_identity.hpp>
namespace stapl {
namespace tuple_ops {

namespace detail {

template<typename Seen, template<typename,typename> class Pred,
  typename Items, typename T>
struct insert_sorted_impl;

template<typename... Seen, template<typename,typename> class Pred,
  typename Head, typename... Tail, typename T>
struct insert_sorted_impl<tuple<Seen...>, Pred, tuple<Head, Tail...>, T>
  : std::conditional<Pred<T, Head>::value,
      type_identity<tuple<Seen..., T, Head, Tail...>>,
      insert_sorted_impl<tuple<Seen..., Head>, Pred, tuple<Tail...>, T>>::type
{ };

template<typename... Seen, template<typename, typename> class Pred, typename T>
struct insert_sorted_impl<tuple<Seen...>, Pred, tuple<>, T>
{
  using type = tuple<Seen..., T>;
};

} // namespace detail

template<typename Items, typename T, template<typename, typename> class Pred>
using insert_sorted =
  typename detail::insert_sorted_impl<tuple<>, Pred, Items, T>;

template<typename Items, typename T>
using insert_sorted_less = insert_sorted<Items, T, type_less>;

} // namespace tuple_ops
} // namespace stapl
#endif // STAPL_UTILITY_TUPLE_INSERT_SORTED_HPP

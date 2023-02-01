/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_CROSS_MAP_HPP
#define STAPL_UTILITY_CROSS_MAP_HPP

#include <stapl/utility/tuple/tuple.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper struct for cross_map which builds the cross product
///        of arguments to pass to the function.
//////////////////////////////////////////////////////////////////////
template<int I>
struct cross_map_impl
{
  template<typename F, typename V, typename... Args>
  static void apply(F&& f, V&& v, Args&&... args)
  {
    auto&& current = stapl::get<I>(v);

    for (auto&& x : current)
      cross_map_impl<I-1>::apply(
        std::forward<F>(f),
        std::forward<V>(v),
        std::forward<decltype(x)>(x),
        std::forward<Args>(args)...
      );
  }
};

template<>
struct cross_map_impl<-1>
{
  template<typename F, typename V, typename... Args>
  static void apply(F&& f, V&& v, Args&&... args)
  {
    f(std::forward<Args>(args)...);
  }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Call a function repeatedly with the set of elements generated
///        by the cross product of all elements in an arbitrary number of
///        containers.
///
/// @param f Function to call. The arity must be equal to the number of
///          containers passed to cross_map.
/// @param v An arbitrary number of containers. They must be iterable
///          using the range-based for construct.
//////////////////////////////////////////////////////////////////////
template<typename F, typename... V>
void cross_map(F&& f, V&&... v)
{
  detail::cross_map_impl<sizeof...(V)-1>::apply(
    std::forward<F>(f), std::forward_as_tuple(v...)
  );
}

} // namespace stapl

#endif // STAPL_UTILITY_CROSS_MAP_HPP

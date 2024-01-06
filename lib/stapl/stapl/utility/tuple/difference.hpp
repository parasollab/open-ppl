/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTIILTY_TUPLE_DIFFERENCE_HPP
#define STAPL_UTIILTY_TUPLE_DIFFERENCE_HPP

#include <type_traits>
#include <stapl/utility/tuple/tuple.hpp>

#include <stapl/utility/tuple/tuple_contains.hpp>
#include <stapl/utility/tuple/homogeneous_tuple.hpp>
#include <stapl/utility/tuple/push_back.hpp>

namespace stapl {
namespace tuple_ops {
namespace result_of {
namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Implementation of difference metafunction. Recursive case.
//////////////////////////////////////////////////////////////////////
template<int CurrentIndex, int LastIndex, typename Larger, typename Smaller,
         typename Running, template<typename...> class Check>
struct difference_impl
{
  using current_element = typename tuple_element<CurrentIndex, Larger>::type;
  using next = typename std::conditional<
    not Check<current_element, Smaller>::value,
    typename tuple_ops::result_of::push_back<Running, current_element>::type,
    Running
  >::type;

  using type = typename difference_impl<
    CurrentIndex+1, LastIndex, Larger, Smaller, next, Check
  >::type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Implementation of difference metafunction. Base case.
//////////////////////////////////////////////////////////////////////
template<int LastIndex, typename Larger, typename Smaller, typename Running,
          template<typename...> class Check>
struct difference_impl<LastIndex, LastIndex, Larger, Smaller, Running, Check>
{
  using type = Running;
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute the result of set difference between
///   two tuples.
///
///  For example, if Larger is <0,2,4> and Smaller is <0,4>, the result
///  will be <2>.
///
/// @tparam Larger Tuple of elements
/// @tparam Smaller Tuple of elements. Must have less or equal elements than
///    Larger.
//////////////////////////////////////////////////////////////////////
template<typename Larger, typename Smaller,
  template<typename...> class Check = tuple_ops::tuple_contains>
struct difference
{
  using type = typename detail::difference_impl<
    0, tuple_size<Larger>::value, Larger, Smaller, tuple<>, Check
  >::type;
};

} // namespace result_of
} // namespace tuple_ops
} // namespace stapl

#endif // STAPL_UTIILTY_TUPLE_DIFFERENCE_HPP

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_TUPLE_SIZE_HPP
#define STAPL_UTILITY_TUPLE_TUPLE_SIZE_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Wrap std::tuple_size and fix deficiency in libstdc++, which
///   doesn't remove cv qualifications.
/// @ingroup Tuple
///
/// This wrapper also reflects value_type and type, and implements
/// value_type().  All of these interfaces are specified in the standard,
/// but are not yet provided by libstd++.
///
/// @todo Monitor https://svn.boost.org/trac/boost/ticket/7192 to
/// determine when we can use std::integral_constant.
//////////////////////////////////////////////////////////////////////
template<typename Tuple>
struct tuple_size
  : public std::tuple_size<
      typename std::remove_cv<Tuple>::type
    >
{
  using value_type = std::size_t;
  using type = std::integral_constant<
                 std::size_t,
                 std::tuple_size<typename std::remove_cv<Tuple>::type>::value>;

  operator std::size_t()
  {
    return std::tuple_size<typename std::remove_cv<Tuple>::type>::value;
  }
};

} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_TUPLE_SIZE_HPP

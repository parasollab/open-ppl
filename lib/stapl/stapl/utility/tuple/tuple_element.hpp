/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_TUPLE_ELEMENT_HPP
#define STAPL_UTILITY_TUPLE_TUPLE_ELEMENT_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Wrap std::tuple_element and fix deficiency in libstdc++, which
///   doesn't remove cv qualifications.
/// @ingroup Tuple
//////////////////////////////////////////////////////////////////////
template<std::size_t N, typename Tuple>
struct tuple_element
  : public std::tuple_element<N, Tuple>
{ };


template<typename std::size_t N, typename Tuple>
struct tuple_element<N, const Tuple>
{
  using type = typename std::add_const<
                 typename std::tuple_element<
                   N, typename std::remove_cv<Tuple>::type>::type>::type;
};

template<typename std::size_t N, typename Tuple>
using tuple_element_t = typename tuple_element<N, Tuple>::type;

} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_TUPLE_ELEMENT_HPP

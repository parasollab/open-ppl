/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_UTILITY_TUPLE_TUPLE_HPP
#define STAPL_UTILITY_TUPLE_TUPLE_HPP

#include <tuple>
#include <utility>

namespace stapl {

/// @file
/// The following using statements are included to provide the possibility of
/// implementing tuples in different ways than the standard.
using std::tuple_cat;
using std::tuple;
using std::ignore;
using std::make_tuple;
using std::tie;
using std::get;
using std::integral_constant;


namespace result_of {

template<typename... Tuples>
struct tuple_cat
{
  using type = decltype(stapl::tuple_cat(std::declval<Tuples>()...));
};

} // namespace result_of

} // namespace stapl

#endif // STAPL_UTILITY_TUPLE_TUPLE_HPP

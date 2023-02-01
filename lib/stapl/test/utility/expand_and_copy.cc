/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/utility/tuple.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/runtime.hpp>

#include "../test_report.hpp"

using namespace stapl;

template<typename T, typename U>
bool is_same_tuple(T&& t, U&& u)
{
  return get<0>(t) == get<0>(u) &&
         get<1>(t) == get<1>(u) &&
         get<2>(t) == get<2>(u);
}

stapl::exit_code stapl_main(int, char*[])
{
  using two_tuple_t = stapl::tuple<std::size_t, std::size_t>;

  using slice1 = tuple_ops::from_index_sequence<index_sequence<0,1>>::type;
  using slice2 = tuple_ops::from_index_sequence<index_sequence<1,2>>::type;

  two_tuple_t tup1(5,10);

  auto res1 = tuple_ops::expand_and_copy<3, slice1>(
    tup1, std::integral_constant<std::size_t, 1>()
  );

  auto res2 = tuple_ops::expand_and_copy<3, slice1>(
    tup1, std::integral_constant<std::size_t, 0>()
  );

  bool passed = is_same_tuple(res1, std::make_tuple(5ul,10ul,1ul));

  STAPL_TEST_REPORT(passed, "Expand with (0,1) slice and fill = 1")

  passed = is_same_tuple(res2, std::make_tuple(5ul,10ul,0ul));

  STAPL_TEST_REPORT(passed, "Expand with (0,1) slice and fill = 0")

  auto res3 = tuple_ops::expand_and_copy<3, slice2>(
    tup1, std::integral_constant<std::size_t, 1>()
  );

  auto res4 = tuple_ops::expand_and_copy<3, slice2>(
    tup1, std::integral_constant<std::size_t, 0>()
  );

  passed = is_same_tuple(res3, std::make_tuple(1ul,5ul,10ul));

  STAPL_TEST_REPORT(passed, "Expand with (1,2) slice and fill = 1")

  passed = is_same_tuple(res4, std::make_tuple(0ul,5ul,10ul));

  STAPL_TEST_REPORT(passed, "Expand with (1,2) slice and fill = 0")

  return EXIT_SUCCESS;
}

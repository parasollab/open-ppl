/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#define REPORT_WITH_COLOR

#include <stapl/runtime.hpp>
#include <stapl/utility/tuple/ensure_tuple.hpp>

#include "../../test_report.hpp"

using namespace stapl;

void test_already_tuple()
{
  using tup = std::tuple<char, float>;
  using result = tuple_ops::ensure_tuple_t<tup>;

  static_assert(std::is_same<tup, result>::value, "ensure_tuple_t<tuple<char,float>>");
}

void test_single_value()
{
  using result = tuple_ops::ensure_tuple_t<char>;
  using expected = std::tuple<char>;

  static_assert(std::is_same<result, expected>::value, "ensure_tuple_t<char>");
}

void test_single_value_runtime()
{
  std::size_t x = 5ul;
  auto result = tuple_ops::ensure_tuple(x);

  STAPL_TEST_REPORT(get<0>(result) == x, "ensure_tuple(size_t)");
}

void test_already_tuple_runtime()
{
  auto tup = make_tuple(5.0, 'c');
  auto result = tuple_ops::ensure_tuple(tup);

  const bool passed =
    get<0>(result) == get<0>(tup) && get<1>(result) == get<1>(tup);

  STAPL_TEST_REPORT(passed, "ensure_tuple(tuple<double, char>)");
}

exit_code stapl_main(int argc, char** argv)
{
  test_single_value_runtime();
  test_already_tuple_runtime();

  return EXIT_SUCCESS;
}

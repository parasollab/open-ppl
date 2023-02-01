/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/utility/unravel_index.hpp>
#include <stapl/runtime/runtime.hpp>

#include "../test_report.hpp"

using namespace stapl;

template<typename T>
bool are_same(T const& t, std::initializer_list<std::size_t> u)
{
  return std::equal(std::begin(t), std::end(t), std::begin(u));
}

void test_1d()
{
  std::vector<std::size_t> shape{2};

  auto u = unravel_index(0, shape);

  STAPL_TEST_REPORT(are_same(u, {0}), "1d 0");

  u = unravel_index(1, shape);
  STAPL_TEST_REPORT(are_same(u, {1}), "1d 1");

}

void test_2d()
{
  std::vector<std::size_t> shape{11, 100};

  auto u = unravel_index(0, shape);

  STAPL_TEST_REPORT(are_same(u, {0, 0}), "2d 0");

  u = unravel_index(1, shape);
  STAPL_TEST_REPORT(are_same(u, {0, 1}), "2d 1");

  u = unravel_index(10, shape);
  STAPL_TEST_REPORT(are_same(u, {0, 10}), "2d 10");

  u = unravel_index(11, shape);
  STAPL_TEST_REPORT(are_same(u, {0, 11}), "2d 11");

  u = unravel_index(99, shape);
  STAPL_TEST_REPORT(are_same(u, {0, 99}), "2d 99");

  u = unravel_index(100, shape);
  STAPL_TEST_REPORT(are_same(u, {1, 0}), "2d 100");

  u = unravel_index(1023, shape);
  STAPL_TEST_REPORT(are_same(u, {10, 23}), "2d 1023");

}

stapl::exit_code stapl_main(int argc, char* argv[])
{

  test_1d();
  test_2d();

  return 0;
}

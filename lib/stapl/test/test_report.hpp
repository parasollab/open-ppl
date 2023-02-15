/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_TEST_REPORT_HPP
#define STAPL_TEST_REPORT_HPP

#include <algorithm>
#include <iostream>

#include <stapl/utility/distributed_value.hpp>
#include <stapl/utility/do_once.hpp>

#define STAPL_TEST_MESSAGE(str)                                                \
  stapl::do_once([&](void) { std::cout << str << "\n"; });

#ifdef REPORT_WITH_COLOR
#  define STAPL_TEST_PASS_STRING "\033[;32mPASSED\033[0m"
#  define STAPL_TEST_FAIL_STRING "\033[;31mFAILED\033[0m"
#else
#  define STAPL_TEST_PASS_STRING "PASSED"
#  define STAPL_TEST_FAIL_STRING "FAILED"
#endif

#define STAPL_TEST_ONE_OF_REPORT(expr, vals, str)                              \
  stapl::do_once([&](void) {                                                   \
    std::cout << str;                                                          \
    if (std::count(std::begin(vals), std::end(vals), expr))                    \
      std::cout << ": " << STAPL_TEST_PASS_STRING << "\n";                     \
    else                                                                       \
      std::cout << ": " << STAPL_TEST_FAIL_STRING << "\n";                     \
  });

#define STAPL_TEST_REPORT(val, str)                                            \
  stapl::do_once([&](void) {                                                   \
    std::cout << str;                                                          \
    if (val)                                                                   \
      std::cout << ": " << STAPL_TEST_PASS_STRING << "\n";                     \
    else                                                                       \
      std::cout << ": " << STAPL_TEST_FAIL_STRING << "\n";                     \
  });

#define STAPL_TEST_ALL_LOCATION_REPORT(val, str)                               \
  do {                                                                         \
    const bool passed = stapl::distributed_value<bool>{ val }                  \
                          .reduce(std::logical_and<bool>{})                    \
                          .get();                                              \
    STAPL_TEST_REPORT(passed, str)                                             \
  } while (0)

#endif // STAPL_TEST_REPORT_HPP

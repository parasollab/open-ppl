/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Test @ref stapl::location_specific_storage.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <stapl/runtime/location_specific_storage.hpp>
#include <iostream>
#include <vector>
#include "../test_utils.h"

const int N = 42;
stapl::location_specific_storage<int>              v(N);
stapl::location_specific_storage<std::vector<int>> vec(2, 1);


struct p_test
: public stapl::p_object
{
  void test_local(void)
  {
    STAPL_RUNTIME_TEST_CHECK(v.get(), N);
    {
      stapl::gang lg;
      STAPL_RUNTIME_TEST_CHECK(v.get(), N);
      ++v.get();
      STAPL_RUNTIME_TEST_CHECK(v.get(), (N + 1));
      STAPL_RUNTIME_TEST_CHECK(vec.get().size(), 2);
      STAPL_RUNTIME_TEST_CHECK(vec.get().at(0), 1);
      STAPL_RUNTIME_TEST_CHECK(vec.get().at(1), 1);
      vec.get() = {5, 6, 7};
    }
    STAPL_RUNTIME_TEST_CHECK(v.get(), N);
    STAPL_RUNTIME_TEST_CHECK(vec.get().size(), 2);
    STAPL_RUNTIME_TEST_CHECK(vec.get().at(0), 1);
    STAPL_RUNTIME_TEST_CHECK(vec.get().at(1), 1);
  }

  void test_nested(void)
  {
    STAPL_RUNTIME_TEST_CHECK(v.get(), N);
    vec.get() = {5, 6, 7};
    {
      stapl::gang g;
      STAPL_RUNTIME_TEST_CHECK(v.get(), N);
      ++v.get();
      STAPL_RUNTIME_TEST_CHECK(v.get(), (N + 1));
      STAPL_RUNTIME_TEST_CHECK(vec.get().size(), 2);
      STAPL_RUNTIME_TEST_CHECK(vec.get().at(0), 1);
      STAPL_RUNTIME_TEST_CHECK(vec.get().at(1), 1);
    }
    STAPL_RUNTIME_TEST_CHECK(v.get(), N);
  }

  void test_destroy(void)
  {
    vec.destroy();
    STAPL_RUNTIME_TEST_CHECK(vec.get().size(), 2);
    STAPL_RUNTIME_TEST_CHECK(vec.get().at(0), 1);
    STAPL_RUNTIME_TEST_CHECK(vec.get().at(1), 1);

    vec.get().push_back(42);
    STAPL_RUNTIME_TEST_CHECK(vec.get().size(), 3);
    STAPL_RUNTIME_TEST_CHECK(vec.get().at(0), 1);
    STAPL_RUNTIME_TEST_CHECK(vec.get().at(1), 1);
    STAPL_RUNTIME_TEST_CHECK(vec.get().at(2), 42);

    vec.destroy();
    STAPL_RUNTIME_TEST_CHECK(vec.get().size(), 2);
    STAPL_RUNTIME_TEST_CHECK(vec.get().at(0), 1);
    STAPL_RUNTIME_TEST_CHECK(vec.get().at(1), 1);
  }

  void test_reset(void)
  {
    vec.destroy();
    STAPL_RUNTIME_TEST_CHECK(vec.get().size(), 2);
    STAPL_RUNTIME_TEST_CHECK(vec.get().at(0), 1);
    STAPL_RUNTIME_TEST_CHECK(vec.get().at(1), 1);

    vec.reset(std::vector<int>{11, 12, 13});
    STAPL_RUNTIME_TEST_CHECK(vec.get().size(), 3);
    STAPL_RUNTIME_TEST_CHECK(vec.get().at(0), 11);
    STAPL_RUNTIME_TEST_CHECK(vec.get().at(1), 12);
    STAPL_RUNTIME_TEST_CHECK(vec.get().at(2), 13);
  }

  void execute(void)
  {
    test_local();
    for (int i=0; i<10; ++i)
      test_nested();
    test_destroy();
    test_reset();
  }
};


stapl::exit_code stapl_main(int, char*[])
{
  p_test pt;
  pt.execute();
#ifndef _TEST_QUIET
  std::cout << stapl::get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}

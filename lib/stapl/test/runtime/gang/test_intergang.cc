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
/// Test intergang communication.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <cstdlib>
#include <functional>
#include "../test_utils.h"

template<typename T>
static void test_sync(T& t, int l)
{
  const int r1 = stapl::sync_rmi(l, t.get_rmi_handle(), &p_test_object::get, 0);
  STAPL_RUNTIME_TEST_CHECK(r1, l);
  stapl::rmi_fence(); // wait for sync_rmi call to finish
}

template<typename T>
static void test_reduction(T& t, const unsigned int RV)
{
  typedef std::plus<std::size_t> plus_op;
  const unsigned int r1 =
    stapl::sync_reduce_rmi(plus_op(), t.get_rmi_handle(),
                           &p_test_object::get_location_id);
  STAPL_RUNTIME_TEST_CHECK(r1, RV);
  stapl::rmi_fence(); // wait for sync_reduce_rmi call to finish
}

struct p_test
: public stapl::p_object
{
  // Tests syncs across gangs
  void test_intergang_sync(void)
  {
    p_test_object g;
    g.set(0, g.get_location_id());
    stapl::rmi_fence(); // wait for set() on all locations

    test_sync(g, g.get_right_neighbor());
    test_sync(g, g.get_left_neighbor());

    if (get_location_id()==0) {
      const int r1 = stapl::sync_rmi(g.get_right_neighbor(), g.get_rmi_handle(),
                                     &p_test_object::get, 0);
      STAPL_RUNTIME_TEST_CHECK(r1, int(g.get_right_neighbor()));
    }
    else if (get_location_id()%2==1) {
      stapl::execute(
        [&g] { test_sync<p_test_object>(g, g.get_left_neighbor()); });
    }
    else {
      stapl::gang lg;
      test_sync(g, g.get_right_neighbor());
    }

    stapl::rmi_fence(); // quiescence before next test

    {
      stapl::gang lg;
      test_sync(g, g.get_left_neighbor());
    }

    {
      stapl::execute(
        [&g] { test_sync<p_test_object>(g, g.get_right_neighbor()); });
    }

    stapl::rmi_fence(); // quiescence before next test
  }

  // Tests reduction across gangs
  void test_intergang_reduction(void)
  {
    p_test_object g;
    typedef std::plus<std::size_t> plus_op;
    unsigned int RV = 0;
    for (unsigned int s = 1; s<g.get_num_locations(); ++s) {
      RV = plus_op()(RV, s);
    }
    const unsigned int r1 =
      stapl::sync_reduce_rmi(plus_op(), g.get_rmi_handle(),
                             &p_test_object::get_location_id);
    STAPL_RUNTIME_TEST_CHECK(r1, RV);

    stapl::rmi_fence(); // quiescence before next test

    if (get_location_id()==0) {
      stapl::gang lg;
      test_reduction(g, RV);
    }
    const unsigned int r2 =
      stapl::sync_reduce_rmi(plus_op(), g.get_rmi_handle(),
                             &p_test_object::get_location_id);
    STAPL_RUNTIME_TEST_CHECK(r2, RV);

    stapl::rmi_fence(); // quiescence before next test

    {
      stapl::gang lg;
      test_reduction(g, RV);
    }
    const unsigned int r3 =
      stapl::sync_reduce_rmi(plus_op(), g.get_rmi_handle(),
                             &p_test_object::get_location_id);
    STAPL_RUNTIME_TEST_CHECK(r3, RV);

    stapl::rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_intergang_sync();
    test_intergang_reduction();
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

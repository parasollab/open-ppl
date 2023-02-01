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
/// Test intragang communication with complex patterns.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <cstdlib>
#include <functional>
#include "../test_utils.h"

static void async_pattern(void)
{
  p_test_object g;

  stapl::async_rmi(g.get_right_neighbor(), g.get_rmi_handle(),
                   &p_test_object::set, 0, g.get_location_id());
  stapl::async_rmi(g.get_right_neighbor(), g.get_rmi_handle(),
                   &p_test_object::test, 0, g.get_location_id());

  stapl::rmi_fence(); // wait for async_rmi calls to finish
  STAPL_RUNTIME_TEST_CHECK(g.get(0), int(g.get_left_neighbor()));
}


// Creates a p_object and makes some local communication
static void comm_pattern(const unsigned int N)
{
  p_test_object g;

  // async_rmi and sync_rmi
  stapl::async_rmi(g.get_right_neighbor(), g.get_rmi_handle(),
                   &p_test_object::set, 0, N*g.get_location_id());
  stapl::async_rmi(g.get_right_neighbor(), g.get_rmi_handle(),
                   &p_test_object::test, 0, N*g.get_location_id());
  const int rg = stapl::sync_rmi(g.get_right_neighbor(), g.get_rmi_handle(),
                                 &p_test_object::get, 0);
  STAPL_RUNTIME_TEST_CHECK(int(N*g.get_location_id()), rg);

  // reduction
  typedef std::plus<unsigned int> plus_op;
  unsigned int RV = 0;
  for (unsigned int s = 1; s<g.get_num_locations(); ++s) {
    RV = plus_op()(RV, s);
  }
  const unsigned int r1 =
    stapl::sync_reduce_rmi(plus_op(), g.get_rmi_handle(),
                           &p_test_object::get_location_id);
  STAPL_RUNTIME_TEST_CHECK(r1, RV);

  stapl::future<unsigned int> r =
    stapl::allreduce_rmi(plus_op(),
                         g.get_rmi_handle(), &p_test_object::get_location_id);
  const unsigned int out = r.get();
  STAPL_RUNTIME_TEST_CHECK(out, RV);

  // create gang and execute again
  if (N!=0) {
    stapl::execute([N] { comm_pattern(N-1); });
  }

  const unsigned int r2 =
    stapl::sync_reduce_rmi(plus_op(), g.get_rmi_handle(),
                           &p_test_object::get_location_id);
  STAPL_RUNTIME_TEST_CHECK(r2, RV);
  stapl::rmi_fence(); // wait for sync_rmi_reduce calls to finish
  STAPL_RUNTIME_TEST_CHECK(int(N*g.get_left_neighbor()), g.get(0));
}


struct p_test
: public stapl::p_object
{
  void test_async(void)
  {
    async_pattern();
  }

  void test_comm_pattern(void)
  {
    for (int i=0; i<5; ++i) {
      comm_pattern(5);
    }
  }

  void execute(void)
  {
    test_async();
    test_comm_pattern();
    test_async();
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

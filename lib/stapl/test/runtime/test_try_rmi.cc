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
/// Test @ref stapl::try_rmi().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include "test_utils.h"

using namespace stapl;

class p_test
: public p_object
{
public:
  // test certain execution
  void test_certain(void)
  {
    p_test_object obj(allow_try_rmi);
    try_rmi(obj.get_left_neighbor(), obj.get_rmi_handle(),
            &p_test_object::set, 0, obj.get_location_id());
    int i = sync_rmi(obj.get_left_neighbor(), obj.get_rmi_handle(),
                     &p_test_object::get, 0);
    async_rmi(obj.get_left_neighbor(), obj.get_rmi_handle(),
              &p_test_object::set_sync, 0);
    block_until([&obj] { return obj.get_sync(0); });
    STAPL_RUNTIME_TEST_CHECK(i, int(obj.get_location_id()));
  }

  // test uncertain execution
  void test_uncertain(void)
  {
    p_test_object obj(allow_try_rmi);
    try_rmi(obj.get_left_neighbor(), obj.get_rmi_handle(),
            &p_test_object::set, 0, obj.get_location_id());
  }

  // test always failed execution
  void test_certain_not(void)
  {
    {
      p_test_object obj(allow_try_rmi);
      try_rmi(obj.get_left_neighbor(), obj.get_rmi_handle(),
              &p_test_object::set, 0, obj.get_location_id());
    }
    {
      p_test_object obj(allow_try_rmi);
      rmi_fence(); // quiescence to detect RMIs that should not have executed
      STAPL_RUNTIME_TEST_CHECK(obj.exists(0), false);
    }
  }

  void execute(void)
  {
    test_certain();
    test_uncertain();
    test_certain_not();
  }
};


exit_code stapl_main(int, char*[])
{
  p_test pt;
  pt.execute();

#ifndef _TEST_QUIET
  std::cout << get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}

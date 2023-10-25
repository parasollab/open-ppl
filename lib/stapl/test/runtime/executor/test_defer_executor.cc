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
/// Test for avoiding executor creation / binding to parent.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <stapl/runtime/executor/scheduler/sched.hpp>
#include <iostream>
#include "simple_executor.hpp"
#include "../test_utils.h"

using namespace stapl;

void dummy_task(void)
{ }

class p_test
: public p_test_object
{
public:
  p_test(void)
  {
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md().try_get_executor()==nullptr
    );
  }

  void execute(void)
  {
    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md().try_get_executor()==nullptr
    );

    auto& ex = get_executor();

    STAPL_RUNTIME_TEST_REQUIRE(
      runtime::this_context::get().get_location_md().try_get_executor()==&ex);
  }
};


exit_code stapl_main(int, char*[])
{
  p_test pt;

  auto f = construct<p_test>(pt.get_rmi_handle(), all_locations);
  rmi_handle::reference r = f.get();

  STAPL_RUNTIME_TEST_REQUIRE(
    runtime::this_context::get().get_location_md().try_get_executor()==nullptr);

  async_rmi(all_locations, r, &p_test::execute);

  rmi_fence();

  STAPL_RUNTIME_TEST_REQUIRE(
    runtime::this_context::get().get_location_md().try_get_executor()==nullptr);

  p_object_delete<p_test> d;
  d(r);

  STAPL_RUNTIME_TEST_REQUIRE(
    runtime::this_context::get().get_location_md().try_get_executor()==nullptr);

#ifndef _TEST_QUIET
  std::cout << get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}

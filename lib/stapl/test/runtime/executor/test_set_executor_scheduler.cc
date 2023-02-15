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
/// Test replacing the scheduler of the @ref stapl::gang_executor.
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
  void execute(void)
  {
    using executor_type = simple_executor<default_scheduler>;
    using task_type     = executor_type::task_type;

    auto* ex = new executor_type;
    ex->add_task(new task_type{&dummy_task});
    get_executor().add_executor(ex);
  }
};


exit_code stapl_main(int, char*[])
{
  set_executor_scheduler(priority_scheduler<10, default_info>{});

  p_test pt;
  pt.execute();
#ifndef _TEST_QUIET
  std::cout << get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}

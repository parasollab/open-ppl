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
/// Test for not creating an executor.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include "../test_utils.h"

using namespace stapl;

class p_test
: public p_test_object
{
public:
  void execute(void)
  { }
};


exit_code stapl_main(int, char*[])
{
  p_test pt;
  pt.execute();

  STAPL_RUNTIME_TEST_REQUIRE(
    runtime::this_context::get().get_location_md().try_get_executor()==nullptr);

#ifndef _TEST_QUIET
  std::cout << get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}

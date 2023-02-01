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
/// Test @ref stapl::restore().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include "test_utils.h"

using namespace stapl;

struct simple_p_object
: public p_object
{
  int operator()(int i) const
  {
    rmi_fence(); // SPMD operation
    return i;
  }

  void foo(int) const
  {
    rmi_fence(); // SPMD operation
  }
};


exit_code stapl_main(int, char*[])
{
  future<rmi_handle::reference> f =
    construct<simple_p_object>(neighbor_locations);

  rmi_handle::reference r = f.get();
  for (int i = 0; i<10; ++i) {
    future<int> fr = restore(r, &simple_p_object::operator(), i);
    STAPL_RUNTIME_TEST_CHECK(fr.get(), i);
  }

  rmi_fence(); // quiescence before next test

  future<void> fv = restore(r, &simple_p_object::foo, 0);
  fv.get();

  rmi_fence(); // quiescence before next test

  p_object_delete<simple_p_object> d;
  d(r);

#ifndef _TEST_QUIET
  std::cout << get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}

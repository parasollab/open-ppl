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
/// Test @ref stapl::rmi_barrier().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include "test_utils.h"

using namespace stapl;

class p_test
: public p_object
{
private:
  p_test_object m_obj;

public:
  // simple test for barrier
  void test_simple(void)
  {
    if (this->get_location_id()%2==0)
      delay(1);
    m_obj.set(0, this->get_location_id());

    rmi_barrier();

    const int l = sync_rmi(m_obj.get_left_neighbor(), m_obj.get_rmi_handle(),
                           &p_test_object::get, 0);
    const int r = sync_rmi(m_obj.get_right_neighbor(), m_obj.get_rmi_handle(),
                           &p_test_object::get, 0);

    typedef unsigned int myuint;
    STAPL_RUNTIME_TEST_CHECK(myuint(l), m_obj.get_left_neighbor());
    STAPL_RUNTIME_TEST_CHECK(myuint(r), m_obj.get_right_neighbor());

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_simple();
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

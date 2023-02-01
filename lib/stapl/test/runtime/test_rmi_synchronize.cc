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
/// Test @ref stapl::rmi_synchronize().
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
  // test with async_rmi calls for rmi_synchronize()
  void test_async_rmi(void)
  {
    if (this->get_location_id()==0) {
      delay(2);
    }
    m_obj.set(0, this->get_location_id() + 1);
    rmi_synchronize();
    async_rmi(m_obj.get_left_neighbor(), m_obj.get_rmi_handle(),
              &p_test_object::test, 0, m_obj.get_left_neighbor() + 1);
    async_rmi(m_obj.get_right_neighbor(), m_obj.get_rmi_handle(),
              &p_test_object::test, 0, m_obj.get_right_neighbor() + 1);

    rmi_fence(); // wait for all RMIs to finish
  }

  void execute(void)
  {
    test_async_rmi();
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

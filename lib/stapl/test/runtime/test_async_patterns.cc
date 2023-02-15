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
/// This test stresses various complex patterns of RMI communication, focusing
/// on data exchanges, common in forwarding. An immature implementation may hang
/// easily on this test.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include "test_utils.h"

using namespace stapl;

struct p_test
: public p_object
{
  const unsigned int m_id;
  const unsigned int m_lt;
  const unsigned int m_rt;
  unsigned int       m_value;

  p_test(void)
  : m_id(this->get_location_id()),
    m_lt((m_id==0) ? (this->get_num_locations()-1) : (m_id - 1)),
    m_rt((m_id==this->get_num_locations()-1) ? 0 : (m_id + 1)),
    m_value(0)
  { this->advance_epoch(); }

  void reset(void)
  { m_value = 0; }

  void set_value(unsigned int v)
  { m_value = v; }

  void set_remote_value(unsigned int location, unsigned int value)
  { async_rmi(location, get_rmi_handle(), &p_test::set_value, value); }

  // Tests exchange of values through async_rmi()
  void test_exchange_values(void)
  {
    reset();
    rmi_fence(); // wait for all locations to reset

    async_rmi(m_rt, get_rmi_handle(), &p_test::set_value, 1);
    block_until([this] { return (this->m_value!=0); });
  }

  // Tests exchange of values in a nested RMI call
  void test_exchange_values_nested(void)
  {
    reset();
    rmi_fence(); // wait for all locations to reset

    async_rmi(m_rt, get_rmi_handle(), &p_test::set_remote_value, m_id, 1);
    block_until([this] { return (this->m_value!=0); });
  }

  void execute(void)
  {
    // Test using default/maximum aggregation/poll_rate on the first pass
    test_exchange_values();
    test_exchange_values_nested();

    // minimal settings on the second pass
    set_aggregation( 1 );
    test_exchange_values();
    test_exchange_values_nested();
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

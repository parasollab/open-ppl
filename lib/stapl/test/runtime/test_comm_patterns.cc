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
/// Test to stress various patterns of RMI communication, focusing on
/// cyclic/nested invocations, common in graph algorithms.
///
/// An immature implementation can hang or deadlock in this test quite easily.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include "test_utils.h"

using namespace stapl;

struct p_test
: public p_object
{
  unsigned int left, right;  // neighbor id's

  p_test(void)
  {
    const unsigned int id = this->get_location_id();
    right = (id==this->get_num_locations()-1) ? 0 : (id + 1);
    left = (id==0) ? (this->get_num_locations()-1) : (id - 1);
    this->advance_epoch();
  }

  unsigned int dummy(void) const noexcept
  { return 1; }

  unsigned int sync_nest(const unsigned int nest)
  {
    const unsigned int n = (this->get_location_id()==0) ? nest-1 : nest;
    if (n!=0) {
      const unsigned int ret =
        sync_rmi(right, get_rmi_handle(), &p_test::sync_nest, n);
      STAPL_RUNTIME_TEST_CHECK(n, ret);
    }
    return nest;
  }

  void async_nest(const unsigned int nest)
  {
    const unsigned int n = (this->get_location_id()==0) ? nest-1 : nest;
    if (n!=0) {
      async_rmi(left, get_rmi_handle(), &p_test::async_nest, n);
      // If only one message can be in-flight at a time, this second call
      // could block.  Although this is legal, make sure the block doesn't
      // eventually cause deadlock (i.e., block without polling).
      async_rmi(left, get_rmi_handle(), &p_test::async_nest, 1);
    }
  }

  // Test centralized communication to a single thread, 0, mixed with
  // dummy calls to stress the internal buffers.
  void test_centralized(void)
  {
    async_rmi(0, get_rmi_handle(), &p_test::sync_nest, 5);
    async_rmi(right, get_rmi_handle(), &p_test::dummy);
    sync_rmi(0, get_rmi_handle(), &p_test::sync_nest, 5);
    async_rmi(left, get_rmi_handle(), &p_test::dummy);

    rmi_fence(); // quiescence before next test
  }

  // Test cyclic sync_rmi communication (i.e., sync_rmi's form a cycle:
  // 0 -> 1 -> ... -> n -> 0), mixed with dummy calls to stress the
  // internal buffers.
  void test_cyclic_sync(void)
  {
    if (this->get_location_id() == 0) {
      async_rmi(right, get_rmi_handle(), &p_test::dummy);
      sync_rmi(right, get_rmi_handle(), &p_test::sync_nest, 3);
      async_rmi(left, get_rmi_handle(), &p_test::dummy);
      sync_rmi(left, get_rmi_handle(), &p_test::sync_nest, 10);
      async_rmi(right, get_rmi_handle(), &p_test::dummy);
      sync_rmi(right, get_rmi_handle(), &p_test::sync_nest, 20);
      async_rmi(left, get_rmi_handle(), &p_test::dummy);
      async_rmi(right, get_rmi_handle(), &p_test::dummy);
    }
    else {
      async_rmi(left, get_rmi_handle(), &p_test::dummy);
      async_rmi(right, get_rmi_handle(), &p_test::dummy);
      sync_rmi(right, get_rmi_handle(), &p_test::sync_nest, 3);
      async_rmi(right, get_rmi_handle(), &p_test::dummy);
      async_rmi(left, get_rmi_handle(), &p_test::dummy);
    }

    rmi_fence(); // quiescence before next test
  }

  // Test cyclic async_rmi communication.
  void test_cyclic_async(void)
  {
    async_rmi(left, get_rmi_handle(), &p_test::async_nest, 1);
    rmi_fence(); // quiescence before next test

    async_rmi(left, get_rmi_handle(), &p_test::async_nest, 2);
    rmi_fence(); // quiescence before next test

    async_rmi(left, get_rmi_handle(), &p_test::async_nest, 10);
    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    // Test using default/maximum aggregation/poll_rate on the first pass
    test_centralized();
    test_cyclic_sync();
    test_cyclic_async();

    // minimal settings on the second pass
    set_aggregation( 1 );
    test_centralized();
    test_cyclic_sync();
    test_cyclic_async();
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

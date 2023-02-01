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
/// Test nested gang creation of 1 location.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <stapl/runtime/utility/functional.hpp>
#include <iostream>
#include <numeric>
#include <cmath>
#include <vector>
#include "../test_utils.h"

struct simple_p_object
: public stapl::p_object
{ };

struct p_test
: public p_test_object
{
  // creates new gangs with just one location
  void test_empty(void)
  {
    stapl::gang g;
    STAPL_RUNTIME_TEST_CHECK(1, stapl::get_num_locations());
    STAPL_RUNTIME_TEST_CHECK(0, stapl::get_location_id());
    {
      stapl::gang g;
      STAPL_RUNTIME_TEST_CHECK(1, stapl::get_num_locations());
      STAPL_RUNTIME_TEST_CHECK(0, stapl::get_location_id());
    }
    STAPL_RUNTIME_TEST_CHECK(1, stapl::get_num_locations());
    STAPL_RUNTIME_TEST_CHECK(0, stapl::get_location_id());
  }

  // creates new gangs with just one location and one object
  void test_one(void)
  {
    stapl::gang g;
    simple_p_object o;
    STAPL_RUNTIME_TEST_CHECK(o.get_num_locations(),
                             stapl::get_num_locations());
    STAPL_RUNTIME_TEST_CHECK(o.get_location_id(),
                             stapl::get_location_id());
    {
      stapl::gang g;
      simple_p_object o;
      STAPL_RUNTIME_TEST_CHECK(o.get_num_locations(),
                               stapl::get_num_locations());
      STAPL_RUNTIME_TEST_CHECK(o.get_location_id(),
                               stapl::get_location_id());
    }
    STAPL_RUNTIME_TEST_CHECK(o.get_num_locations(),
                             stapl::get_num_locations());
    STAPL_RUNTIME_TEST_CHECK(o.get_location_id(),
                             stapl::get_location_id());
  }

  // creates new gangs with one heap-allocated object on the outer
  void test_outer(void)
  {
    simple_p_object *p = 0;
    {
      stapl::gang g;
      p = new simple_p_object;
      STAPL_RUNTIME_TEST_CHECK(p->get_num_locations(),
                               stapl::get_num_locations());
      STAPL_RUNTIME_TEST_CHECK(p->get_location_id(),
                               stapl::get_location_id());
      {
        stapl::gang g;
        STAPL_RUNTIME_TEST_CHECK(1, stapl::get_num_locations());
        STAPL_RUNTIME_TEST_CHECK(0, stapl::get_location_id());
      }
      STAPL_RUNTIME_TEST_CHECK(p->get_num_locations(),
                               stapl::get_num_locations());
      STAPL_RUNTIME_TEST_CHECK(p->get_location_id(),
                               stapl::get_location_id());
    }
    stapl::p_object_delete<simple_p_object> d;
    d(p);
    stapl::rmi_fence(); // wait for p_object_delete to finish
  }

  // creates new gangs with one heap-allocated object in the inner
  void test_inner(void)
  {
    simple_p_object *p = 0;
    {
      stapl::gang g;
      STAPL_RUNTIME_TEST_CHECK(1, stapl::get_num_locations());
      STAPL_RUNTIME_TEST_CHECK(0, stapl::get_location_id());
      {
        stapl::gang g;
        p = new simple_p_object;
        STAPL_RUNTIME_TEST_CHECK(p->get_num_locations(),
                                 stapl::get_num_locations());
        STAPL_RUNTIME_TEST_CHECK(p->get_location_id(),
                                 stapl::get_location_id());
      }
      STAPL_RUNTIME_TEST_CHECK(1, stapl::get_num_locations());
      STAPL_RUNTIME_TEST_CHECK(0, stapl::get_location_id());
    }
    stapl::p_object_delete<simple_p_object> d;
    d(p);
    stapl::rmi_fence(); // wait for p_object_delete to finish
  }

  void execute(void)
  {
    test_empty();
    test_one();
    test_outer();
    test_inner();
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

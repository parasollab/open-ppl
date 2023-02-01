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
/// Test intragang communication.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <stapl/runtime/utility/functional.hpp>
#include <iostream>
#include <numeric>
#include <cmath>
#include "../test_utils.h"

struct p_test
: public p_test_object
{
  // creates a new gang with all locations
  void test_one(void)
  {
    typedef stapl::runtime::identity<unsigned int, unsigned int> map_fun_type;

    const unsigned int num_locs = stapl::get_num_locations();

    // do async_rmi
    if (stapl::get_location_id()!=0) {
      delay(1);
    }
    stapl::gang g(num_locs, map_fun_type(), map_fun_type());
    const unsigned int num_locs_inner = stapl::get_num_locations();
    p_test_object o;
    if (stapl::get_location_id()==0) {
      // send to last location
      stapl::async_rmi(num_locs_inner-1, o.get_rmi_handle(),
                       &p_test_object::set, 1, 1);
      STAPL_RUNTIME_TEST_CHECK(1, stapl::sync_rmi(num_locs_inner-1,
                                                  o.get_rmi_handle(),
                                                  &p_test_object::get, 1));
    }
    stapl::rmi_fence(); // wait for async_rmi calls to finish
    STAPL_RUNTIME_TEST_CHECK(1, stapl::sync_rmi(num_locs_inner-1,
                                                o.get_rmi_handle(),
                                                &p_test_object::get, 1));
    stapl::rmi_fence(); // wait for sync_rmi calls to finish
  }

  // creates a new gang with all locations
  void test_all(void)
  {
    const unsigned int num_locs = stapl::get_num_locations();

    typedef stapl::runtime::identity<unsigned int, unsigned int> map_fun_type;

    if (stapl::get_location_id()!=0 && stapl::get_location_id()!=num_locs/2) {
      delay(1);
    }
    stapl::gang g(num_locs, map_fun_type(), map_fun_type());
    p_test_object o;
    if (stapl::get_location_id()==num_locs/2) {
      // send to all locations
      stapl::unordered::async_rmi(stapl::all_locations, o.get_rmi_handle(),
                                  &p_test_object::set, 1, 1);
    }
    stapl::rmi_fence(); // wait for async_rmi calls to finish
    STAPL_RUNTIME_TEST_CHECK(1, o.get(1));
  }

  void execute(void)
  {
    test_one();
    test_all();
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

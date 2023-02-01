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
/// Test if the @ref stapl::affinity_tag returned by @ref stapl::get_affinity()
/// is unique among all locations.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <algorithm>
#include <iostream>
#include "test_utils.h"

using namespace stapl;

class p_test
: public p_object
{
public:
  affinity_tag get_affinity(void) const noexcept
  { return stapl::get_affinity(); }
};


void check_affinity(void)
{
  p_test pt;
  if (get_location_id()==0) {
    futures<affinity_tag> f =
      opaque_rmi(all_locations, pt.get_rmi_handle(), &p_test::get_affinity);

    std::vector<affinity_tag> v = f.get();

    STAPL_RUNTIME_TEST_REQUIRE((std::unique(v.begin(), v.end())==v.end()));
  }
  rmi_fence(); // wait for all opaque_rmi calls to finish

  if (get_available_levels()>0) {
    execute(&check_affinity);
  }
}


exit_code stapl_main(int, char*[])
{
  check_affinity();

#ifndef _TEST_QUIET
  std::cout << get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}

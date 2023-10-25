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
/// Test deleting location metadata through deleting its last
/// @ref stapl::p_object.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <cmath>
#include "test_utils.h"

using namespace stapl;

struct simple_p_object
: public p_object
{ };

struct map_wf
{
  typedef unsigned int result_type;

  unsigned int operator()(unsigned int n) const
  { return (n/2); }
};

struct reverse_map_wf
{
  typedef unsigned int result_type;

  unsigned int operator()(unsigned int n) const
  { return (2*n); }
};

// even locations only
rmi_handle::reference object_in_gang(void)
{
  const unsigned int num_locs = get_num_locations();
  gang g(std::ceil(num_locs/2.0), map_wf(), reverse_map_wf());
  simple_p_object* p = new simple_p_object;
  return p->get_rmi_handle();
}

class p_test
: public p_object
{
private:
  unsigned int right;

  void kill(rmi_handle::reference r)
  {
    p_object_delete<simple_p_object> d;
    d(r);
  }

public:
  p_test(void)
  {
    const unsigned int lid = this->get_location_id();
    const unsigned int nlocs = this->get_num_locations();
    right = ( lid == nlocs - 1 ) ? 0 : lid + 1;
    this->advance_epoch();
  }

  // Creates and kills an object in a gang
  void test_gang_object(void)
  {
    if (this->get_location_id()%2==0) {
      rmi_handle::reference ref = object_in_gang();
      if (this->get_location_id()==0) {
        kill(ref);
      }
    }

    rmi_fence(); // quiescence before next test
  }

  // Creates an object which is killed by a location outside the initial gang
  void test_gang_object_3rd_party(void)
  {
    if (this->get_location_id()%2==0) {
      rmi_handle::reference ref = object_in_gang();
      if (this->get_location_id()==0)
        async_rmi(right, this->get_rmi_handle(), &p_test::kill, ref);
    }
  }

  void execute(void)
  {
    test_gang_object();
    test_gang_object_3rd_party(); // it must be the last test to detect if gangs
                                  // are correctly garbage collected
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

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
/// Test @ref stapl::async_rmi() / @ref stapl::unordered::async_rmi() to range
/// of locations.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <functional>
#include "test_utils.h"

using namespace stapl;

struct p_test
: public p_object
{
  unsigned int m_num;

  p_test(void)
  : m_num(0)
  { this->advance_epoch(); }

  unsigned int get(void)
  { return this->get_location_id(); }

  void inc(void)
  { ++m_num; }

  void check(unsigned int n)
  { STAPL_RUNTIME_TEST_CHECK(n, m_num); }

  void reset(void)
  {
    m_num = 0;
    rmi_fence(); // wait for m_num to be set on all locations
  }

  void test_single(void)
  {
    std::vector<unsigned int> v;
    for (auto i = 0u; i < this->get_num_locations(); ++i)
      v.push_back(i);

    if (this->get_location_id()==0) {
      unordered::async_rmi(location_range(v), this->get_rmi_handle(),
                           &p_test::inc);
    }

    rmi_fence(); // wait for all async_rmi calls to finish before checking
    STAPL_RUNTIME_TEST_CHECK(1, m_num);

    rmi_fence(); // quiescence before next test
  }

  // test unordered::async_rmi(all_locations)
  void test_all_locations_unordered(void)
  {
    std::vector<unsigned int> v;
    for (auto i = 0u; i < this->get_num_locations(); ++i)
      v.push_back(i);

    for (unsigned int i=0; i<this->get_num_locations(); ++i) {
      reset();

      if (this->get_location_id()==i) {
        unordered::async_rmi(location_range(v), this->get_rmi_handle(),
                             &p_test::inc);
      }

      rmi_fence(); // wait for all async_rmi calls to finish before checking
      STAPL_RUNTIME_TEST_CHECK(1, m_num);
    }

    rmi_fence(); // quiescence before next test
  }

  // test unordered::async_rmi(location_range)
  void test_even_locations_unordered(void)
  {
    std::vector<unsigned int> v;
    for (auto i = 0u; i < this->get_num_locations(); i+=2)
      v.push_back(i);

    for (unsigned int i=0; i<this->get_num_locations(); ++i) {
      reset();

      if (this->get_location_id()==i) {
        unordered::async_rmi(location_range(v), this->get_rmi_handle(),
                             &p_test::inc);
      }

      rmi_fence(); // wait for all async_rmi calls to finish before checking

      if (this->get_location_id()%2==0) {
        STAPL_RUNTIME_TEST_CHECK(1, m_num);
      }
      else {
        STAPL_RUNTIME_TEST_CHECK(0, m_num);
      }
    }

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_single();
    test_all_locations_unordered();
    test_even_locations_unordered();
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

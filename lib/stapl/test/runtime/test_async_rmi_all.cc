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
/// Test @ref stapl::async_rmi() / @ref stapl::unordered::async_rmi() to all
/// locations.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <functional>
#include "test_utils.h"

using namespace stapl;

struct p_test
: public p_object
{
  unsigned int        m_num;
  std::vector<double> m_v;

  p_test(void)
  : m_num(0)
  { this->advance_epoch(); }

  unsigned int get(void)
  { return this->get_location_id(); }

  void inc(void)
  { ++m_num; }

  void check(unsigned int n)
  { STAPL_RUNTIME_TEST_CHECK(n, m_num); }

  void set_vector(std::vector<double>&& v)
  { m_v = std::move(v); }

  void reset(void)
  {
    m_num = 0;
    m_v   = {};
    rmi_fence(); // wait for reset() to finish on all locations
  }

  // test async_rmi(all_locations)
  void test_all_locations(void)
  {
    reset();

    async_rmi(all_locations, this->get_rmi_handle(), &p_test::inc);

    rmi_fence(); // wait for async_rmi calls to finish
    STAPL_RUNTIME_TEST_CHECK(this->get_num_locations(), m_num);

    rmi_fence(); // quiescence before next test
  }

  // test async_rmi(all_locations) with nested for-loops
  void test_all_locations_nested(void)
  {
    for (unsigned int i=0; i<this->get_num_locations(); ++i) {
      reset();

      if (this->get_location_id()==i) {
        for (unsigned int j=0; j<this->get_num_locations(); ++j) {
          async_rmi(j, this->get_rmi_handle(), &p_test::inc);
        }
        async_rmi(all_locations, this->get_rmi_handle(), &p_test::check, 1);
        async_rmi(all_locations, this->get_rmi_handle(), &p_test::inc);
        async_rmi(all_locations, this->get_rmi_handle(), &p_test::check, 2);
      }

      rmi_fence(); // wait for RMI calls to finish
      STAPL_RUNTIME_TEST_CHECK(2, m_num);
    }

    rmi_fence(); // quiescence before next test
  }

  // test opaque_rmi(all_locations)/async_rmi(all_locations)
  void test_combination(void)
  {
    reset();

    async_rmi(all_locations, this->get_rmi_handle(), &p_test::inc);
    opaque_rmi(all_locations, this->get_rmi_handle(), &p_test::inc).get();
    async_rmi(all_locations, this->get_rmi_handle(), &p_test::inc);
    async_rmi(all_locations, this->get_rmi_handle(), &p_test::inc);
    opaque_rmi(all_locations, this->get_rmi_handle(), &p_test::inc).get();

    rmi_fence(); // wait for RMI calls to finish
    STAPL_RUNTIME_TEST_CHECK((5*this->get_num_locations()), m_num);

    rmi_fence(); // quiescence before next test
  }

  // test opaque_rmi(all_locations)/async_rmi(all_locations) from each location
  void test_combination_each_location(void)
  {
    for (unsigned int i=0; i<this->get_num_locations(); ++i) {
      reset();

      if (this->get_location_id()==i) {
        async_rmi(all_locations, this->get_rmi_handle(), &p_test::inc);
        opaque_rmi(all_locations, this->get_rmi_handle(), &p_test::inc).get();
        async_rmi(all_locations, this->get_rmi_handle(), &p_test::inc);
        opaque_rmi(all_locations, this->get_rmi_handle(),
                   &p_test::check, 3).get();
        async_rmi(all_locations, this->get_rmi_handle(), &p_test::inc);
        opaque_rmi(all_locations, this->get_rmi_handle(), &p_test::inc).get();
      }

      rmi_fence(); // wait for RMI calls to finish
      STAPL_RUNTIME_TEST_CHECK(5, m_num);
    }

    rmi_fence(); // quiescence before next test
  }

  // test unordered::async_rmi(all_locations)
  void test_all_locations_unordered(void)
  {
    reset();

    unordered::async_rmi(all_locations, this->get_rmi_handle(), &p_test::inc);

    rmi_fence(); // wait for all async_rmi calls to finish before checking
    STAPL_RUNTIME_TEST_CHECK(this->get_num_locations(), m_num);

    rmi_fence(); // quiescence before next test
  }

  // test async_rmi()/unordered::async_rmi(all_locations)
  void test_combination_unordered(void)
  {
    for (unsigned int i=0; i<this->get_num_locations(); ++i) {
      reset();

      if (this->get_location_id()==i) {
        for (unsigned int j=0; j<this->get_num_locations(); ++j) {
          async_rmi(j, this->get_rmi_handle(), &p_test::inc);
        }
        unordered::async_rmi(all_locations, this->get_rmi_handle(),
                             &p_test::inc);
      }

      rmi_fence(); // wait for all async_rmi calls to finish before checking
      STAPL_RUNTIME_TEST_CHECK(2, m_num);
    }

    rmi_fence(); // quiescence before next test
  }

  // test unordered::async_rmi(all_locations) with vector moving
  void test_unordered_vector(void)
  {
    for (unsigned int i=0; i<this->get_num_locations(); ++i) {

      const std::vector<double> v(93750, i);

      reset();

      if (this->get_location_id()==i) {
        auto t = v;
        unordered::async_rmi(all_locations, this->get_rmi_handle(),
                             &p_test::set_vector, std::move(t));
      }

      rmi_fence(); // wait for all async_rmi calls to finish before checking

      STAPL_RUNTIME_TEST_CHECK(v.size(), m_v.size());
      STAPL_RUNTIME_TEST_REQUIRE(std::equal(v.begin(), v.end(), m_v.begin()));
    }

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_all_locations();
    test_all_locations_nested();
    test_combination();
    test_combination_each_location();

    test_all_locations_unordered();
    test_combination_unordered();

    test_unordered_vector();
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

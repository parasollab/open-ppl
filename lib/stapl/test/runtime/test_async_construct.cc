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
/// Test @ref stapl::async_construct().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <stapl/runtime/random_location_generator.hpp>
#include <algorithm>
#include <iostream>
#include <vector>
#include "test_utils.h"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Creates @p nlocs random location IDs.
//////////////////////////////////////////////////////////////////////
std::vector<unsigned int> random_locations(unsigned int nlocs)
{
  std::vector<unsigned int> v;
  v.reserve(nlocs);
  random_location_generator rng;
  for (unsigned int i = 0; i < nlocs; ++i) {
    unsigned int lid = 0;
    do {
      lid = rng();
    } while (std::find(v.begin(), v.end(), lid)!=v.end());
    v.push_back(lid);
  }
  return v;
}


//////////////////////////////////////////////////////////////////////
/// @brief Dummy distributed object.
//////////////////////////////////////////////////////////////////////
class simple_p_object
: public p_object
{
public:
  simple_p_object(void) = default;

  template<typename T>
  simple_p_object(T const&)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Test harness distributed object.
//////////////////////////////////////////////////////////////////////
class p_test
: public p_object
{
public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Test for creating a @ref simple_p_object to specific locations of
  ///        the current gang.
  //////////////////////////////////////////////////////////////////////
  void test_construct_range(void)
  {
    if (this->get_location_id()==0) {
      // generate set of unique random location ids
      std::vector<unsigned int> v;
      if (this->get_num_locations()==1)
        v = { 0 };
      else
        v = random_locations(this->get_num_locations()/2);

      // create object
#if 0
      async_construct<simple_p_object>(
        [](simple_p_object* p) { delete p; },
        location_range(v), 2);
#else
      // address gforge task 1444
      promise<void> p;
      auto f = p.get_future();
      async_construct<simple_p_object>(
        std::bind(
          [](promise<void>& pr, simple_p_object* p)
          {
            if (p->get_location_id()==0)
              pr.set_value();
            delete p;
          }, std::move(p), std::placeholders::_1),
        location_range(v), 2);
      f.get();
#endif
    }

    rmi_fence(); // quiescence before next test
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Test for creating a @ref simple_p_object to all locations of
  ///        another distributed object.
  //////////////////////////////////////////////////////////////////////
  void test_construct_handle(void)
  {
    if (this->get_location_id()==0) {
      // create object
#if 0
      async_construct<simple_p_object>(
        [](simple_p_object* p) { delete p; },
        this->get_rmi_handle(), all_locations);
#else
      promise<void> p;
      auto f = p.get_future();
      async_construct<simple_p_object>(
        std::bind(
          [](promise<void>& pr, simple_p_object* p)
          {
            if (p->get_location_id()==0)
              pr.set_value();
            delete p;
          }, std::move(p), std::placeholders::_1),
        this->get_rmi_handle(), all_locations);
      f.get();
#endif
    }

    rmi_fence(); // quiescence before next test
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Test for creating a @ref simple_p_object to specific locations of
  ///        another distributed object.
  //////////////////////////////////////////////////////////////////////
  void test_construct_handle_range(void)
  {
    if (this->get_location_id()==0) {
      // generate set of unique random location ids
      std::vector<unsigned int> v;
      if (this->get_num_locations()==1)
        v = { 0 };
      else
        v = random_locations(this->get_num_locations()/2);

      // create object
#if 0
      async_construct<simple_p_object>(
        [](simple_p_object* p) { delete p; },
        this->get_rmi_handle(), location_range(v));
#else
      promise<void> p;
      auto f = p.get_future();
      async_construct<simple_p_object>(
        std::bind(
          [](promise<void>& pr, simple_p_object* p)
          {
            if (p->get_location_id()==0)
              pr.set_value();
            delete p;
          }, std::move(p), std::placeholders::_1),
        this->get_rmi_handle(), location_range(v));
      f.get();
#endif
    }

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_construct_range();
    test_construct_handle();
    test_construct_handle_range();
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

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
/// Test @ref stapl::construct().
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

  simple_p_object(rmi_handle::reference const& r)
  {
    STAPL_RUNTIME_TEST_REQUIRE(compare_gangs(this->get_rmi_handle(), r)==1);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Test harness distributed object.
//////////////////////////////////////////////////////////////////////
class p_test
: public p_object
{
public:
  //////////////////////////////////////////////////////////////////////
  /// @brief RMI target for creating a new @ref simple_p_object.
  //////////////////////////////////////////////////////////////////////
  void simple_create(rmi_handle::const_reference const& h)
  {
    rmi_handle::const_reference htmp = h;
    std::vector<unsigned int> v{0};
    future<rmi_handle::reference> f =
      construct<simple_p_object>(std::move(htmp), location_range(std::move(v)));

    p_object_delete<simple_p_object> d;

    rmi_handle::reference r = f.get();
    d(r);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Test for creating a @ref simple_p_object to all neighbor locations.
  //////////////////////////////////////////////////////////////////////
  void test_construct_neighbor(void)
  {
    if (this->get_location_id()==0) {
      // create object
      future<rmi_handle::reference> f =
        construct<simple_p_object>(neighbor_locations);

      // delete it
      rmi_handle::reference r = f.get();
      p_object_delete<simple_p_object> d;
      d(r);
    }

    rmi_fence(); // quiescence before next test
  }

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

      // create objects
      future<rmi_handle::reference> f0 =
        construct<simple_p_object>(location_range(v), 2);
      future<rmi_handle::reference> f1 =
        construct<simple_p_object>(location_range(v), 2);
      future<rmi_handle::reference> f2 =
        construct<simple_p_object>(location_range(std::move(v)), 2);

      // delete objects
      p_object_delete<simple_p_object> d;
      rmi_handle::reference r0 = f0.get();
      d(r0);
      rmi_handle::reference r1 = f1.get();
      d(r1);
      rmi_handle::reference r2 = f2.get();
      d(r2);
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
      // create objects
      future<rmi_handle::reference> f0 =
        construct<simple_p_object>(this->get_rmi_handle(), all_locations);
      future<rmi_handle::reference> f1 =
        construct<simple_p_object>(this->get_rmi_handle(), all_locations);

      rmi_handle::reference r0 = f0.get();
      rmi_handle::reference r1 = f1.get();

      future<rmi_handle::reference> f2 =
        construct<simple_p_object>(r0, all_locations, r0);

      // delete objects
      p_object_delete<simple_p_object> d;
      d(r0);
      d(r1);
      rmi_handle::reference r2 = f2.get();
      d(r2);
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

      // create objects
      future<rmi_handle::reference> f0 =
        construct<simple_p_object>(this->get_rmi_handle(), location_range(v));
      future<rmi_handle::reference> f1 =
        construct<simple_p_object>(this->get_rmi_handle(), location_range(v));
      future<rmi_handle::reference> f2 =
        construct<simple_p_object>(this->get_rmi_handle(),
                                   location_range(std::move(v)));

      // delete objects
      p_object_delete<simple_p_object> d;
      rmi_handle::reference r0 = f0.get();
      d(r0);
      rmi_handle::reference r1 = f1.get();
      d(r1);
      rmi_handle::reference r2 = f2.get();
      d(r2);
    }

    rmi_fence(); // quiescence before next test
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Test for creating a @ref simple_p_object to specific locations of
  ///        another distributed object that does not have metadata on all
  ///        locations.
  //////////////////////////////////////////////////////////////////////
  void test_construct_handle_remote(void)
  {
    if (this->get_location_id()==0) {
      gang g;
      simple_p_object o;
      async_rmi(this->get_num_locations() - 1, this->get_rmi_handle(),
                &p_test::simple_create, o.get_rmi_handle());
      rmi_fence(); // wait for async_rmi to finish
    }
    rmi_fence(); // quiescence before next test
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Test for creating a @ref simple_p_object to one location with use
  ///        of @c future<T>::async_then().
  //////////////////////////////////////////////////////////////////////
  void test_async_then(void)
  {
    std::vector<unsigned int> v = { stapl::get_location_id() };
    future<rmi_handle::reference> f0 =
      construct<simple_p_object>(this->get_rmi_handle(), location_range(v));

    f0.async_then(
      [](future<rmi_handle::reference> f)
      {
        rmi_handle::reference r = f.get();
        p_object_delete<simple_p_object> d;
        d(r);
      });

    rmi_fence(); // wait for async_then() to finish
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Test for creating a @ref simple_p_object to one location with use
  ///        of @c future<T>::async_then().
  //////////////////////////////////////////////////////////////////////
  void test_async_then_object(void)
  {
    if (this->get_location_id() == 0) {
      gang g;

      future<rmi_handle::reference> f0 =
        construct<simple_p_object>(this->get_rmi_handle(), all_locations);

      f0.async_then(
        [](future<rmi_handle::reference> f)
        {
          rmi_handle::reference r = f.get();
          p_object_delete<simple_p_object> d;
          d(r);
        });

      rmi_fence(); // wait for async_then() to finish
    }

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_construct_neighbor();
    test_construct_range();
    test_construct_handle();
    test_construct_handle_range();
    test_construct_handle_remote();
    test_async_then();
    test_async_then_object();
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

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
/// Test creating gangs collectively and early exit through
/// @ref stapl::gang::leave().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <stapl/runtime/utility/functional.hpp>
#include <iostream>
#include <numeric>
#include <cmath>
#include <vector>
#include "../test_utils.h"

using namespace stapl;

struct reverse_fun
{
  typedef unsigned int result_type;

  const unsigned int size;

  constexpr explicit reverse_fun(const unsigned int s) noexcept
  : size(s)
  { }

  constexpr unsigned int operator()(unsigned int n) const noexcept
  { return ((size-1)-n); }

  void define_type(typer& t)
  { t.member(size); }

  friend bool operator==(reverse_fun const& x, reverse_fun const& y) noexcept
  { return (x.size==y.size); }
};


struct even_fun
{
  typedef unsigned int result_type;

  constexpr unsigned int operator()(unsigned int n) const noexcept
  { return (n/2); }
};


struct reverse_even_fun
{
  typedef unsigned int result_type;

  constexpr unsigned int operator()(unsigned int n) const noexcept
  { return (2*n); }
};


struct arbitrary
{
  typedef unsigned int result_type;

  constexpr unsigned int operator()(unsigned int n) const noexcept
  { return (n==0 ? 0 : 1); }
};


struct reverse_arbitrary
{
  typedef unsigned int result_type;

  const unsigned int m_uint;

  constexpr explicit reverse_arbitrary(unsigned int i) noexcept
  : m_uint(i)
  { }

  constexpr unsigned int operator()(unsigned int n) const noexcept
  { return (n==0 ? 0 : m_uint); }

  void define_type(typer& t)
  { t.member(m_uint); }

  friend constexpr bool operator==(reverse_arbitrary const& x,
                                   reverse_arbitrary const& y) noexcept
  { return (x.m_uint==y.m_uint); }
};


struct p_test
: public p_test_object
{
  // creates new gangs with just one location
  void test_one(void)
  {
    gang g;
    STAPL_RUNTIME_TEST_CHECK(1, stapl::get_num_locations());
    STAPL_RUNTIME_TEST_CHECK(0, stapl::get_location_id());
  }

  // creates a new gang with all locations
  void test_all(void)
  {
    const unsigned int lid      = stapl::get_location_id();
    const unsigned int num_locs = stapl::get_num_locations();

    typedef runtime::identity<unsigned int, unsigned int> map_fun_type;
    gang g(num_locs, map_fun_type(), map_fun_type());
    STAPL_RUNTIME_TEST_CHECK(num_locs, stapl::get_num_locations());
    STAPL_RUNTIME_TEST_CHECK(lid, stapl::get_location_id());
  }

  // creates a new gang with all locations reversed
  void test_reversed(void)
  {
    const unsigned int lid      = stapl::get_location_id();
    const unsigned int num_locs = stapl::get_num_locations();

    gang g(num_locs, reverse_fun(num_locs), reverse_fun(num_locs));
    const unsigned int new_lid = stapl::get_location_id();
    STAPL_RUNTIME_TEST_CHECK(num_locs, stapl::get_num_locations());
    STAPL_RUNTIME_TEST_CHECK(lid, (stapl::get_num_locations() - new_lid - 1));
  }

  // creates a new gang with all the even locations
  void test_even(void)
  {
    const unsigned int lid      = stapl::get_location_id();
    const unsigned int num_locs = stapl::get_num_locations();

    // create partition
    if (this->get_location_id()%2==0) {
      gang g(std::ceil(num_locs/2.0), even_fun(), reverse_even_fun());
      STAPL_RUNTIME_TEST_CHECK(std::ceil(num_locs/2.0),
                               stapl::get_num_locations());
      STAPL_RUNTIME_TEST_CHECK(lid,
                               reverse_even_fun()(stapl::get_location_id()));
    }
  }

  // creates a new gang with all locations a few times
  void test_fast(void)
  {
    typedef runtime::identity<unsigned int, unsigned int> map_fun_type;

    const unsigned int N = 1000;
    const unsigned int num_locs = stapl::get_num_locations();

    // create partition
    for (unsigned int i = 0; i<N; ++i) {
      gang g(num_locs, map_fun_type(), map_fun_type());
    }
  }

  // creates different disjointed gangs, all including 0
  void test_zero(void)
  {
    const unsigned int lid      = stapl::get_location_id();
    const unsigned int num_locs = stapl::get_num_locations();

    if (lid==0) {
      for (unsigned int i=1; i<num_locs; ++i) {
        gang g(2, arbitrary(), reverse_arbitrary(i));
        STAPL_RUNTIME_TEST_CHECK(2, stapl::get_num_locations());
        STAPL_RUNTIME_TEST_CHECK(0, stapl::get_location_id());
      }
    }
    else {
      gang g(2, arbitrary(), reverse_arbitrary(lid));
      STAPL_RUNTIME_TEST_CHECK(2, stapl::get_num_locations());
      STAPL_RUNTIME_TEST_CHECK(1, stapl::get_location_id());
    }
  }

  // creates new gangs with just one location and exits in the middle
  void test_one_leave(void)
  {
    const unsigned int lid      = stapl::get_location_id();
    const unsigned int num_locs = stapl::get_num_locations();

    gang g;
    STAPL_RUNTIME_TEST_CHECK(1, stapl::get_num_locations());
    STAPL_RUNTIME_TEST_CHECK(0, stapl::get_location_id());

    g.leave();
    STAPL_RUNTIME_TEST_CHECK(num_locs, stapl::get_num_locations());
    STAPL_RUNTIME_TEST_CHECK(lid, stapl::get_location_id());
  }

  // creates a new gang with all the even locations and exits in the middle
  void test_leave(void)
  {
    const unsigned int lid      = stapl::get_location_id();
    const unsigned int num_locs = stapl::get_num_locations();

    // create partition
    if (this->get_location_id()%2==0) {
      gang g(std::ceil(num_locs/2.0), even_fun(), reverse_even_fun());
      STAPL_RUNTIME_TEST_CHECK(std::ceil(num_locs/2.0),
                               stapl::get_num_locations());
      STAPL_RUNTIME_TEST_CHECK(lid,
                               reverse_even_fun()(stapl::get_location_id()));

      g.leave();
      STAPL_RUNTIME_TEST_CHECK(num_locs, stapl::get_num_locations());
      STAPL_RUNTIME_TEST_CHECK(lid, stapl::get_location_id());
    }
  }

  void execute(void)
  {
    test_one();
    test_all();
    test_reversed();
    test_even();
    test_fast();
    test_zero();
    test_one_leave();
    test_leave();
  }
};


exit_code stapl_main(int, char*[])
{
  p_test pt;
  pt.execute();
#ifndef _TEST_QUIET
  std::cout << stapl::get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}

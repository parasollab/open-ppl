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
/// Test @ref stapl::make_range() and @ref stapl::make_range_n().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>
#include "test_utils.h"

using namespace stapl;

class p_test
: public p_object
{
private:
  unsigned int m_right;

public:
  p_test(void)
  {
    const unsigned int id = this->get_location_id();
    m_right = (id == this->get_num_locations() - 1) ? 0 : id + 1;
    this->advance_epoch();
  }

  template<typename Range, typename T>
  void test_same_ptr(Range const& r, raw_ptr<T> p)
  {
    auto const& t = *std::begin(r);
    T const* tp   = p;
    STAPL_RUNTIME_TEST_REQUIRE(&t==tp);
  }

  template<typename Range, typename T>
  void test_range(Range const& r, T const& t)
  {
    STAPL_RUNTIME_TEST_CHECK(r.size(), t.size());
    STAPL_RUNTIME_TEST_REQUIRE(
      std::equal(std::begin(r), std::end(r), std::begin(t))
    );
  }

  template<typename Range, typename T>
  void test_diff_ptr(Range const& r, raw_ptr<T> p)
  {
    auto const& t = *std::begin(r);
    T const* tp   = p;
    STAPL_RUNTIME_TEST_REQUIRE(&t!=tp);
  }

  void test_make_range(void)
  {
    std::vector<int> v1 = { 0, 1, 2, 3, 4, 5, 6 };

    auto first = std::begin(v1);
    ++first;

    auto last = first;
    std::advance(last, (v1.size() - 1));

    const std::vector<int> v2{first, last};

    auto r = make_range(first, last);

    // local test
    test_range(r, v2);

    // copy range
    async_rmi(m_right, this->get_rmi_handle(),
              &p_test::test_range<decltype(r), decltype(v2)>, r, v2);

    // move range
    async_rmi(m_right, this->get_rmi_handle(),
              &p_test::test_range<decltype(r), decltype(v2)>, std::move(r), v2);

    v1.clear();

    rmi_fence(); // quiescence before next test
  }

  void test_make_range_n(void)
  {
    std::vector<int> v1 = { 0, 1, 2, 3, 4, 5, 6 };

    auto first = std::begin(v1);
    ++first;

    auto last = first;
    std::advance(last, (v1.size() - 1));

    const std::vector<int> v2{first, last};

    auto r = make_range_n(first, (v1.size() - 1));

    // local test
    test_range(r, v2);

    // copy range
    async_rmi(m_right, this->get_rmi_handle(),
              &p_test::test_range<decltype(r), decltype(v2)>, r, v2);

    // move range
    async_rmi(m_right, this->get_rmi_handle(),
              &p_test::test_range<decltype(r), decltype(v2)>, std::move(r), v2);

    v1.clear();

    rmi_fence(); // quiescence before next test
  }

  void test_self(void)
  {
    std::vector<int> v = { 0 };

    auto r = make_range(std::begin(v), std::end(v));

    // local test
    test_same_ptr(r, raw_ptr<int>{&v[0]});

    // copy range
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::test_diff_ptr<decltype(r), int>,
              r, raw_ptr<int>{&v[0]});

    // move range
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::test_diff_ptr<decltype(r), int>,
              std::move(r), raw_ptr<int>{&v[0]});

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_make_range();
    test_make_range_n();
    test_self();
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

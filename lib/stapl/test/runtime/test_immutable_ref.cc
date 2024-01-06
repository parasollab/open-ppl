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
/// Test @ref stapl::immutable_ref().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <algorithm>
#include <iostream>
#include <iterator>
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

  template<typename T>
  void test(T const& t1, T const& t2) const
  {
    STAPL_RUNTIME_TEST_CHECK(t1, t2);
  }

  template<typename T>
  void test_range(T const& t1, T const& t2)
  {
    STAPL_RUNTIME_TEST_CHECK(t1.size(), t2.size());
    STAPL_RUNTIME_TEST_REQUIRE(
      std::equal(std::begin(t1), std::end(t1), std::begin(t2))
    );
  }

  template<typename T>
  void test_same_ptr(T const& t, raw_ptr<T> p)
  {
    T const* tp = p;
    STAPL_RUNTIME_TEST_REQUIRE(&t==tp);
  }

  template<typename T>
  void test_diff_ptr(T const& t, raw_ptr<T> p)
  {
    T const* tp = p;
    STAPL_RUNTIME_TEST_REQUIRE(&t!=tp);
  }

  void test_int(void)
  {
    int i = 42;

    async_rmi(m_right, this->get_rmi_handle(),
              &p_test::test<decltype(i)>, immutable_ref(i), i);

    rmi_fence(); // quiescence before next test
  }

  void test_vector(void)
  {
    std::vector<int> v = { 0, 1, 2, 3, 4, 5, 6, 0 };

    async_rmi(m_right, this->get_rmi_handle(),
              &p_test::test_range<decltype(v)>, immutable_ref(v), v);

    rmi_fence(); // quiescence before next test
  }

  void test_self(void)
  {
    std::vector<int> v = { 0, 1, 2, 3, 4, 5, 6, 0 };

    // immutable_ref() by move
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::test_same_ptr<decltype(v)>,
              immutable_ref(v), raw_ptr<decltype(v)>(&v));

    // immutable_ref by copy
    auto r = immutable_ref(v);
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::test_same_ptr<decltype(v)>,
              r, raw_ptr<decltype(v)>(&v));

    rmi_fence(); // quiescence before next test
  }

  void test_self_copy(void)
  {
    std::vector<p_test*> v = { this };

    // immutable_ref() by move
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::test_diff_ptr<decltype(v)>,
              immutable_ref(v), raw_ptr<decltype(v)>(&v));

    // immutable_ref by copy
    auto r = immutable_ref(v);
    async_rmi(this->get_location_id(), this->get_rmi_handle(),
              &p_test::test_diff_ptr<decltype(v)>,
              r, raw_ptr<decltype(v)>(&v));

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_int();
    test_vector();
    test_self();
    test_self_copy();
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

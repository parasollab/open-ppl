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
/// Test @ref stapl::sync_rmi().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <limits>
#include <vector>
#include "test_utils.h"

using namespace stapl;

struct p_test
: public p_object
{
  unsigned int left, right; // neighbor id's

  p_test(void)
  {
    const unsigned int myid = this->get_location_id();
    right = (myid == this->get_num_locations() - 1) ? 0 : myid + 1;
    left = (myid == 0) ? this->get_num_locations() - 1 : myid - 1;
    this->advance_epoch();
  }

  void f(void) const
  { }

  template<typename T>
  T identity(const T t)
  { return t; }

  void test_fundamental(void)
  {
    sync_rmi(right, this->get_rmi_handle(), &p_test::f);

    const bool b = sync_rmi(right, this->get_rmi_handle(),
                            &p_test::identity<bool>, false);
    STAPL_RUNTIME_TEST_CHECK(false, b);

    const char c1 = sync_rmi(right, this->get_rmi_handle(),
                             &p_test::identity<char>, '1');
    STAPL_RUNTIME_TEST_CHECK('1', c1);
    const signed char c2 = sync_rmi(right, this->get_rmi_handle(),
                                    &p_test::identity<signed char>, '2');
    STAPL_RUNTIME_TEST_CHECK('2', c2);
    const unsigned char c3 = sync_rmi(right, this->get_rmi_handle(),
                                      &p_test::identity<unsigned char>, '3');
    STAPL_RUNTIME_TEST_CHECK('3', c3);

    const short s1 = sync_rmi(right, this->get_rmi_handle(),
                              &p_test::identity<short>, 1);
    STAPL_RUNTIME_TEST_CHECK(1, s1);
    const signed short s2 = sync_rmi(right, this->get_rmi_handle(),
                                     &p_test::identity<signed short>, 2);
    STAPL_RUNTIME_TEST_CHECK(2, s2);
    const unsigned short s3 = sync_rmi(right, this->get_rmi_handle(),
                                       &p_test::identity<unsigned short>, 3);
    STAPL_RUNTIME_TEST_CHECK(3, s3);

    const int i1 = sync_rmi(right, this->get_rmi_handle(),
                            &p_test::identity<int>, 1);
    STAPL_RUNTIME_TEST_CHECK(1, i1);
    const signed int i2 = sync_rmi(right, this->get_rmi_handle(),
                                   &p_test::identity<signed int>, 2);
    STAPL_RUNTIME_TEST_CHECK(2, i2);
    const unsigned int i3 = sync_rmi(right, this->get_rmi_handle(),
                                     &p_test::identity<unsigned int>, 3u);
    STAPL_RUNTIME_TEST_CHECK(3, i3);

    const long l1 = sync_rmi(right, this->get_rmi_handle(),
                             &p_test::identity<long>, 1l);
    STAPL_RUNTIME_TEST_CHECK(1, l1);
    const signed long l2 = sync_rmi(right, this->get_rmi_handle(),
                                    &p_test::identity<signed long>, 2l);
    STAPL_RUNTIME_TEST_CHECK(2, l2);
    const unsigned long l3 = sync_rmi(right, this->get_rmi_handle(),
                                      &p_test::identity<unsigned long>, 3ul);
    STAPL_RUNTIME_TEST_CHECK(3, l3);

    const float f = sync_rmi(right, this->get_rmi_handle(),
                             &p_test::identity<float>, 1.0f);
    STAPL_RUNTIME_TEST_CHECK((1.0 - f < std::numeric_limits<float>::epsilon()),
                             true);

    const double d = sync_rmi(right, this->get_rmi_handle(),
                              &p_test::identity<double>, 1.1);
    STAPL_RUNTIME_TEST_CHECK((1.1 - d < std::numeric_limits<double>::epsilon()),
                             true);

    const long double ld = sync_rmi(right, this->get_rmi_handle(),
                                    &p_test::identity<long double>, 1.2l);
    STAPL_RUNTIME_TEST_CHECK(
        (1.2 - ld < std::numeric_limits<long double>::epsilon()), true);

    rmi_fence(); // wait for all RMIs to finish
  }

  // Tests an arbitrary object with no pointers
  stack_object stack_object_return(stack_object const& t)
  {
    stack_object tt = t;
    STAPL_RUNTIME_TEST_CHECK(tt, stack_object(right));
    return stack_object(right + 1);
  }

  void test_stack_object(void)
  {
    stack_object o0(this->get_location_id());
    stack_object o1(this->get_location_id() + 1);
    stack_object s = sync_rmi(left, this->get_rmi_handle(),
                                      &p_test::stack_object_return, o0);
    STAPL_RUNTIME_TEST_CHECK(s, o1);

    rmi_fence(); // wait for all RMIs to finish
  }

  // Tests an arbitrary object with pointers
  dynamic_object dynamic_object_return(dynamic_object const& t)
  {
    dynamic_object tt = t;
    STAPL_RUNTIME_TEST_CHECK(tt, dynamic_object(left));
    return dynamic_object(left + 1);
  }

  void test_dynamic_object(void)
  {
    dynamic_object o0(this->get_location_id());
    dynamic_object o1(this->get_location_id() + 1);
    dynamic_object d = sync_rmi(right, this->get_rmi_handle(),
                                        &p_test::dynamic_object_return, o0);
    STAPL_RUNTIME_TEST_CHECK(d, o1);

    rmi_fence(); // wait for all RMIs to finish
  }

  // Tests an std::vector<int> without overlapping
  std::vector<int> vector_return(std::size_t s)
  {
    std::vector<int> v;
    make_vector(v, s);
    STAPL_RUNTIME_TEST_CHECK(check_vector(v, s), true);
    return v;
  }

  void test_vector(std::vector<int>& v, const std::size_t s)
  {
    STAPL_RUNTIME_TEST_CHECK(check_vector(v, s), true);
  }

  void make_vector(std::vector<int>& v, const std::size_t size)
  {
    for (int i = size - 1; i >= 0; --i)
      v.push_back(i);
  }

  bool check_vector(std::vector<int>& v, const std::size_t size)
  {
    if (v.size() != size)
      return false;
    for (int start = size - 1; start >= 0; start--) {
      if (v[size - start - 1] != start)
        return false;
    }
    return true;
  }

  void test_vector(void)
  {
    const std::size_t N = 1000;
    for (std::size_t i = N / 100; i < N; ++i) {
      std::vector<int> v = sync_rmi(right, this->get_rmi_handle(),
                                           &p_test::vector_return, i);
      test_vector(v, i);
    }

    rmi_fence(); // wait for all RMIs to finish
  }

  void execute(void)
  {
    test_fundamental();
    test_vector();
    test_stack_object();
    test_dynamic_object();
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

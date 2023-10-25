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
/// Test @ref stapl::bind_rmi().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <stapl/utility/tuple.hpp>
#include "test_utils.h"

using namespace stapl;

struct p_test
: public p_test_object
{
  int m_n;

  p_test(void)
  : m_n(0)
  { this->advance_epoch(); }

  void test_void(void)
  { --m_n; }

  void test_bind_rmi_void(void)
  {
    const int N = 20000;
    m_n = N;
    rmi_fence(); // wait for all locations to set m_n

    auto tunnel = bind_rmi(&p_test::test_void, this->get_rmi_handle());

    for (int i=0; i<N; ++i)
      tunnel(get_left_neighbor());

    tunnel.flush();

    rmi_fence(); // wait for RMI call completion
    STAPL_RUNTIME_TEST_CHECK(0, m_n);
  }

  void test_empty(empty)
  { --m_n; }

  void send_empty(empty e)
  {
    --m_n;
    async_rmi(get_left_neighbor(), this->get_rmi_handle(),
              &p_test::test_empty, e);
  }

  void test_bind_rmi_empty(void)
  {
    const int N = 20000;
    m_n = 2*N;
    rmi_fence(); // wait for all locations to set m_n

    auto tunnel = bind_rmi(&p_test::send_empty, this->get_rmi_handle());

    for (int i=0; i<N; ++i )
      tunnel(get_left_neighbor(), empty{});

    tunnel.flush();

    rmi_fence(); // wait for RMI call completion
    STAPL_RUNTIME_TEST_CHECK(0, m_n);
  }

  void send_bool(bool b)
  {
    --m_n;
    STAPL_RUNTIME_TEST_CHECK(false, b);
    async_rmi(get_right_neighbor(), this->get_rmi_handle(),
              &p_test::test_bool, b);
  }

  void test_bool(bool b)
  {
    --m_n;
    STAPL_RUNTIME_TEST_CHECK(false, b);
  }

  void test_bind_rmi_bool(void)
  {
    const int N = 20000;
    m_n = 2*N;
    rmi_fence(); // wait for all locations to set m_n

    auto tunnel = bind_rmi(&p_test::send_bool, this->get_rmi_handle());

    for (int i=0; i<N; ++i )
      tunnel(get_left_neighbor(), false);

    tunnel.flush();

    rmi_fence(); // wait for async_rmi call completion
    STAPL_RUNTIME_TEST_CHECK(0, m_n);
  }

  void send_int(int i)
  {
    --m_n;
    STAPL_RUNTIME_TEST_CHECK(42, i);
    async_rmi(get_right_neighbor(), this->get_rmi_handle(),
              &p_test::test_int, i);
  }

  void test_int(int i)
  {
    --m_n;
    STAPL_RUNTIME_TEST_CHECK(42, i);
  }

  void test_bind_rmi_int(void)
  {
    const int N = 20000;
    m_n = 2*N;
    rmi_fence(); // wait for all locations to set m_n

    auto tunnel = bind_rmi(&p_test::send_int, this->get_rmi_handle());

    for (int i=0; i<N; ++i)
      tunnel(get_left_neighbor(), 42);

    tunnel.flush();

    rmi_fence(); // wait for async_rmi call completion
    STAPL_RUNTIME_TEST_CHECK(0, m_n);
  }

  void send_vector(const std::vector<int>& v, const std::size_t s)
  {
    STAPL_RUNTIME_TEST_CHECK(check_vector(v, s), true);
    async_rmi(get_right_neighbor(), this->get_rmi_handle(),
              &p_test::test_vector, v, s);
  }

  void test_vector(const std::vector<int>& v, const std::size_t s)
  {
    STAPL_RUNTIME_TEST_CHECK(check_vector(v, s), true);
  }

  std::vector<int> make_vector(const std::size_t size)
  {
    std::vector<int> v;
    for (int i=size-1; i>=0; --i)
      v.push_back(i);
    return v;
  }

  bool check_vector(const std::vector<int>& v, const std::size_t size)
  {
    if (v.size()!=size)
      return false;
    for (int start = size-1; start>=0; start--) {
      if (v[size-start-1]!=start)
        return false;
    }
    return true;
  }

  void test_bind_rmi_vector(void)
  {
    const unsigned int N = 200;

    auto tunnel = bind_rmi(&p_test::send_vector, this->get_rmi_handle());

    for (unsigned int i = 1; i<N; ++i) {
      auto v = make_vector(i);

      tunnel(get_left_neighbor(), v, v.size());
    }

    tunnel.flush();

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_bind_rmi_void();
    test_bind_rmi_empty();
    test_bind_rmi_bool();
    test_bind_rmi_int();
    test_bind_rmi_vector();
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

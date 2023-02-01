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
/// Test @ref stapl::async_rmi().
///
/// This file also tests if combining is performed correctly.
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

  void test_void2(void)
  { ++m_n; }

  void send_void(void)
  {
    --m_n;
    async_rmi(get_left_neighbor(), this->get_rmi_handle(), &p_test::test_void);
  }

  void send_void2(void)
  {
    ++m_n;
    async_rmi(get_left_neighbor(), this->get_rmi_handle(), &p_test::test_void2);
  }

  void async_test_void(void)
  {
    const int N = 20000;
    m_n = 2*N;
    rmi_fence(); // wait for all locations to set m_n
    for (int i=0; i<N; ++i )
      async_rmi(get_left_neighbor(), this->get_rmi_handle(),
                &p_test::send_void);

    rmi_fence(); // wait for async_rmi call completion
    STAPL_RUNTIME_TEST_CHECK(0, m_n);
  }

  void async_test_void2(void)
  {
    const int N = 20000;
    m_n = 0;
    rmi_fence(); // wait for all locations to set m_n
    for (int i=0; i<N; ++i ) {
      async_rmi(get_left_neighbor(), this->get_rmi_handle(),
                &p_test::send_void);
      async_rmi(get_left_neighbor(), this->get_rmi_handle(),
                &p_test::send_void2);
    }

    rmi_fence(); // wait for async_rmi call completion
    STAPL_RUNTIME_TEST_CHECK(0, m_n);
  }

  void test_empty(empty)
  { --m_n; }

  void test_empty2(empty)
  { ++m_n; }

  void send_empty(empty e)
  {
    --m_n;
    async_rmi(get_left_neighbor(), this->get_rmi_handle(),
              &p_test::test_empty, e);
  }

  void send_empty2(empty e)
  {
    --m_n;
    async_rmi(get_left_neighbor(), this->get_rmi_handle(),
              &p_test::test_empty2, e);
  }

  void async_test_empty(void)
  {
    const int N = 20000;
    m_n = 2*N;
    rmi_fence(); // wait for all locations to set m_n
    for (int i=0; i<N; ++i )
      async_rmi(get_left_neighbor(), this->get_rmi_handle(),
                &p_test::send_empty, empty());

    rmi_fence(); // wait for async_rmi call completion
    STAPL_RUNTIME_TEST_CHECK(0, m_n);
  }

  void async_test_empty2(void)
  {
    const int N = 20000;
    m_n = 0;
    rmi_fence(); // wait for all locations to set m_n
    for (int i=0; i<N; ++i )
      async_rmi(get_left_neighbor(), this->get_rmi_handle(),
                &p_test::send_empty2, empty());

    rmi_fence(); // wait for async_rmi call completion
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

  void async_test_bool(void)
  {
    const int N = 20000;
    m_n = 2*N;
    rmi_fence(); // wait for all locations to set m_n
    for (int i=0; i<N; ++i )
      async_rmi(get_left_neighbor(), this->get_rmi_handle(),
                &p_test::send_bool, false);

    rmi_fence(); // wait for async_rmi call completion
    STAPL_RUNTIME_TEST_CHECK(0, m_n);
  }

#if 0
  // TODO range functions over combined arguments support is not in trunk yet
  void send_multi_bool(ArgumentSetIterator1<bool>& begin,
                       ArgumentSetIterator1<bool>& end)
  {
    for (; begin!=end; ++begin) {
      bool v = begin.getArg0();
      --_n;
      STAPL_RUNTIME_TEST_CHECK(false, v);
      async_rmi(get_right_neighbor(), get_rmi_handle(),
                &p_test::test_bool, v);
    }
  }

  void test_multi_bool(ArgumentSetIterator1<bool>& begin,
                       ArgumentSetIterator1<bool>& end)
  {
    for (; begin!=end; ++begin) {
      --_n;
      STAPL_RUNTIME_TEST_CHECK(false, begin.getArg0());
    }
  }
#endif

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

  void async_test_int(void)
  {
    const int N = 20000;
    m_n = 2*N;
    rmi_fence(); // wait for all locations to set m_n
    for (int i=0; i<N; ++i)
      async_rmi(get_left_neighbor(), this->get_rmi_handle(),
                &p_test::send_int, 42);

    rmi_fence(); // wait for async_rmi call completion
    STAPL_RUNTIME_TEST_CHECK(0, m_n);
  }

#if 0
  // TODO range functions over combined arguments support is not in trunk yet
  void send_multi_int(ArgumentSetIterator1<int>& begin,
                      ArgumentSetIterator1<int>& end)
  {
    for (; begin!=end; ++begin) {
      int v = begin.getArg0();
      --_n;
      STAPL_RUNTIME_TEST_CHECK(42, v);
      async_rmi(get_right_neighbor(), get_rmi_handle(),
                &p_test::test_int, v);
    }
  }

  void test_multi_int(ArgumentSetIterator1<int>& begin,
                      ArgumentSetIterator1<int>& end)
  {
    for (; begin!=end; ++begin) {
      --_n;
      STAPL_RUNTIME_TEST_CHECK(42, begin.getArg0());
    }
  }
#endif

  void send_int2(int i, int j)
  {
    --m_n;
    STAPL_RUNTIME_TEST_CHECK(i, (j-1));
    async_rmi(get_right_neighbor(), this->get_rmi_handle(),
              &p_test::test_int2, i, j);
  }

  void test_int2(int i, int j)
  {
    --m_n;
    STAPL_RUNTIME_TEST_CHECK(i, (j-1));
  }

  void async_test_int2(void)
  {
    const int N = 7;
    m_n = 2*N;
    rmi_fence(); // wait for all locations to set m_n
    for (int i=0; i<N; ++i)
      async_rmi(get_left_neighbor(), this->get_rmi_handle(),
                &p_test::send_int2, i, i+1);

    rmi_fence(); // wait for async_rmi call completion
    STAPL_RUNTIME_TEST_CHECK(0, m_n);
  }

  void send_int3(int i, int j, int k)
  {
    --m_n;
    STAPL_RUNTIME_TEST_CHECK(i, (k+j));
    async_rmi(get_right_neighbor(), this->get_rmi_handle(),
              &p_test::test_int3, i, j, k);
  }

  void test_int3(int i, int j, int k)
  {
    --m_n;
    STAPL_RUNTIME_TEST_CHECK(i, (k+j));
  }

  void async_test_int3(void)
  {
    const int N = 7;
    m_n = 2*N;
    rmi_fence(); // wait for all locations to set m_n
    for (int i=0; i<N; ++i)
      async_rmi(get_left_neighbor(), this->get_rmi_handle(),
                &p_test::send_int3, (3*i+1), (i+1), (2*i));

    rmi_fence(); // wait for async_rmi call completion
    STAPL_RUNTIME_TEST_CHECK(0, m_n);
  }


  void send_dynamic(dynamic_object& d, int j)
  {
    --m_n;
    STAPL_RUNTIME_TEST_CHECK(d, dynamic_object(j));
    async_rmi(get_right_neighbor(), this->get_rmi_handle(),
              &p_test::test_dynamic, d, j);
  }

  void test_dynamic(dynamic_object& d, int j)
  {
    --m_n;
    STAPL_RUNTIME_TEST_CHECK(d, dynamic_object(j));
  }

  void async_test_dynamic(void)
  {
    const int N = 200;
    m_n = 2*N;
    rmi_fence(); // wait for all locations to set m_n
    for (int i=0; i<N; ++i) {
      dynamic_object d(i);
      async_rmi(get_left_neighbor(), this->get_rmi_handle(),
                &p_test::send_dynamic, d, i);
    }

    rmi_fence(); // wait for async_rmi call completion
    STAPL_RUNTIME_TEST_CHECK(0, m_n);
  }

  void send_dynamic2(dynamic_object& d1, dynamic_object& d2, int j)
  {
    --m_n;
    STAPL_RUNTIME_TEST_CHECK(d1, dynamic_object(j));
    STAPL_RUNTIME_TEST_CHECK(d2, dynamic_object(j + 10));
    async_rmi(get_right_neighbor(), this->get_rmi_handle(),
              &p_test::test_dynamic2, d1, d2, j);
  }

  void test_dynamic2(dynamic_object& d1, dynamic_object& d2, int j)
  {
    --m_n;
    STAPL_RUNTIME_TEST_CHECK(d1, dynamic_object(j));
    STAPL_RUNTIME_TEST_CHECK(d2, dynamic_object(j + 10));
  }

  void async_test_dynamic2(void)
  {
    const int N = 200;
    m_n = 2*N;
    rmi_fence(); // wait for all locations to set m_n
    for (int i=0; i<N; ++i) {
      dynamic_object d1(i);
      dynamic_object d2(i+10);
      async_rmi(get_left_neighbor(), this->get_rmi_handle(),
                &p_test::send_dynamic2, d1, d2, i);
    }

    rmi_fence(); // wait for async_rmi call completion
    STAPL_RUNTIME_TEST_CHECK(0, m_n);
  }

  void send_vector(std::vector<int>& v, const std::size_t s)
  {
    STAPL_RUNTIME_TEST_CHECK(check_vector(v, s), true);
    async_rmi(get_right_neighbor(), this->get_rmi_handle(),
              &p_test::test_vector, v, s);
  }

  void test_vector(std::vector<int>& v, const std::size_t s)
  {
    STAPL_RUNTIME_TEST_CHECK(check_vector(v, s), true);
  }

#if 0
  // TODO range functions over combined arguments support is not in trunk yet
  void send_multi_vector(
    ArgumentSetIterator2<std::vector<int>&, std::size_t>& begin,
    ArgumentSetIterator2<std::vector<int>&, std::size_t>& end)
  {
    for (; begin!=end; ++begin) {
      std::vector<int>& v = begin.getArg0();
      size_t& s = begin.getArg1();
      STAPL_RUNTIME_TEST_CHECK(check_vector(v, s), true);
      async_rmi(get_right_neighbor(), this->get_rmi_handle(),
                &p_test::test_vector, v, s);
    }
  }

  void test_multi_vector(
    ArgumentSetIterator2<std::vector<int>&, std::size_t>& begin,
    ArgumentSetIterator2<std::vector<int>&, std::size_t>& end)
  {
    for (; begin!=end; ++begin) {
      std::vector<int>& v = begin.getArg0();
      size_t& s = begin.getArg1();
      STAPL_RUNTIME_TEST_CHECK(check_vector(v, s), true);
    }
  }
#endif

  void make_vector(std::vector<int>& v, const std::size_t size)
  {
    for (int i=size-1; i>=0; --i)
      v.push_back(i);
  }

  bool check_vector(std::vector<int>& v, const std::size_t size)
  {
    if (v.size()!=size) return false;
    for (int start = size-1; start>=0; start--) {
      if (v[size-start-1]!=start) return false;
    }
    return true;
  }

  void async_test_vector(void)
  {
    const unsigned int N = 200;
    for (unsigned int i = 1; i<N; ++i) {
      std::vector<int> v;
      make_vector(v, i);
      async_rmi(get_left_neighbor(), this->get_rmi_handle(),
                &p_test::send_vector, v, v.size());
    }

    rmi_fence(); // quiescence before next test
  }

  void send_tuple(const std::size_t i, tuple<int, int, int> const& t)
  {
    STAPL_RUNTIME_TEST_CHECK(check_tuple(t, i), true);
    async_rmi(get_right_neighbor(), this->get_rmi_handle(),
              &p_test::test_tuple, i, t);
    async_rmi(get_left_neighbor(), this->get_rmi_handle(),
              &p_test::test_tuple, i, t);
  }

  void test_tuple(const std::size_t i, tuple<int, int, int> const& t)
  {
    STAPL_RUNTIME_TEST_CHECK(check_tuple(t, i), true);
  }

  bool check_tuple(tuple<int, int, int> const& t, const std::size_t i)
  {
    const int j = static_cast<int>(i);
    return (t == make_tuple(j, j+2, j+5));
  }

  void async_test_tuple(void)
  {
    const unsigned int N = 200;
    m_n = 0;
    for (unsigned int i=0; i<N; ++i) {
      tuple<int, int, int> t(m_n, m_n+2, m_n+5);
      async_rmi(get_left_neighbor(), this->get_rmi_handle(),
                &p_test::send_tuple, m_n, t);
      async_rmi(get_right_neighbor(), this->get_rmi_handle(),
                &p_test::send_tuple, m_n, t);
      ++m_n;
    }

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    async_test_void();
    async_test_void2();
    async_test_empty();
    async_test_empty2();
    async_test_bool();

#if 0
    // TODO range functions over combined arguments support is not in trunk yet
    set_combine_function(get_rmi_handle(),
                         &p_test::send_bool, &p_test::send_multi_bool);
    set_combine_function(get_rmi_handle(),
                         &p_test::test_bool, &p_test::test_multi_bool);
#endif

    async_test_bool();
    async_test_int();


#if 0
    // TODO range functions over combined arguments support is not in trunk yet
    set_combine_function(get_rmi_handle(),
                         &p_test::send_int, &p_test::send_multi_int);
    set_combine_function(get_rmi_handle(),
                         &p_test::test_int, &p_test::test_multi_int);
#endif

    async_test_int();
    async_test_int2();
    async_test_int3();
    async_test_dynamic();
    async_test_dynamic2();
    async_test_vector();

#if 0
    // TODO range functions over combined arguments support is not in trunk yet
    set_combine_function(get_rmi_handle(),
                         &p_test::send_vector, &p_test::send_multi_vector);
    set_combine_function(get_rmi_handle(),
                         &p_test::test_vector, &p_test::test_multi_vector);
#endif

    async_test_vector();
    async_test_tuple();
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

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
/// Test @ref stapl::promise.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <vector>
#include "test_utils.h"

using namespace stapl;

struct p_test
: public p_object
{
  unsigned int right;

  p_test(void)
  : right(this->get_location_id()==(this->get_num_locations()-1)
          ? 0
          : this->get_location_id()+1)
  { this->advance_epoch(); }

  template<typename T>
  void use_promise_none(promise<T> p)
  { p.set_value(); }

  template<typename T>
  void use_promise(promise<T> p, T const& t)
  { p.set_value(t); }

  void test_fundamental(void)
  {
    {
      typedef void value_type;
      promise<value_type> p;
      future<value_type> f = p.get_future();
      async_rmi(right, this->get_rmi_handle(),
                &p_test::use_promise_none<value_type>, std::move(p));
      f.get();
    }

    {
      typedef bool value_type;

      const value_type value = false;
      promise<value_type> p;
      future<value_type> f = p.get_future();
      async_rmi(right, this->get_rmi_handle(),
                &p_test::use_promise<value_type>, std::move(p), value);
      STAPL_RUNTIME_TEST_CHECK(value, f.get());
    }

    {
      typedef char value_type;

      const value_type value = 'a';
      promise<value_type> p;
      future<value_type> f = p.get_future();
      async_rmi(right, this->get_rmi_handle(),
                &p_test::use_promise<value_type>, std::move(p), value);
      STAPL_RUNTIME_TEST_CHECK(value, f.get());
    }

    {
      typedef int value_type;

      const value_type value = 42;
      promise<value_type> p;
      future<value_type> f = p.get_future();
      async_rmi(right, this->get_rmi_handle(),
                &p_test::use_promise<value_type>, std::move(p), value);
      STAPL_RUNTIME_TEST_CHECK(value, f.get());
    }

    {
      typedef long value_type;

      const value_type value = 42;
      promise<value_type> p;
      future<value_type> f = p.get_future();
      async_rmi(right, this->get_rmi_handle(),
                &p_test::use_promise<value_type>, std::move(p), value);
      STAPL_RUNTIME_TEST_CHECK(value, f.get());
    }

    {
      typedef long long value_type;

      const value_type value = 42;
      promise<value_type> p;
      future<value_type> f = p.get_future();
      async_rmi(right, this->get_rmi_handle(),
                &p_test::use_promise<value_type>, std::move(p), value);
      STAPL_RUNTIME_TEST_CHECK(value, f.get());
    }

    {
      typedef float value_type;

      const value_type value = 42.0;
      promise<value_type> p;
      future<value_type> f = p.get_future();
      async_rmi(right, this->get_rmi_handle(),
                &p_test::use_promise<value_type>, std::move(p), value);
      STAPL_RUNTIME_TEST_REQUIRE(
        (value - f.get()) < std::numeric_limits<value_type>::epsilon());
    }

    {
      typedef double value_type;

      const value_type value = 42.0;
      promise<value_type> p;
      future<value_type> f = p.get_future();
      async_rmi(right, this->get_rmi_handle(),
                &p_test::use_promise<value_type>, std::move(p), value);
      STAPL_RUNTIME_TEST_REQUIRE(
        (value - f.get()) < std::numeric_limits<value_type>::epsilon());
    }

    rmi_fence(); // quiescence before next test
  }

  template<typename T>
  bool check_vector(std::vector<T> const& v, const std::size_t size)
  {
    if (v.size() != size)
      return false;
    for (std::size_t i = size; v.size() != size; --i) {
      if (v[size - i] != T(i))
        return false;
    }
    return true;
  }

  template<typename T>
  void make_vector(std::vector<T>& v, const std::size_t size)
  {
    for (std::size_t i = size; v.size() != size; --i)
      v.push_back(T(i));
  }

  template<typename T>
  void vector_return(const std::size_t s, promise<std::vector<T>> p)
  {
    std::vector<T> v;
    make_vector(v, s);
    STAPL_RUNTIME_TEST_CHECK(check_vector(v, s), true);
    p.set_value(std::move(v));
  }

  void test_vector(void)
  {
    const std::size_t N = 100;

    {
      typedef int                     value_type;
      typedef std::vector<value_type> vector_type;

      for (std::size_t i = 0; i < N; ++i) {
        promise<vector_type> p;
        future<vector_type> f = p.get_future();
        async_rmi(right, this->get_rmi_handle(),
                  &p_test::vector_return<value_type>, i, std::move(p));
        const vector_type v = f.get();
        STAPL_RUNTIME_TEST_CHECK(check_vector(v, i), true);
      }
    }

    {
      typedef pod_stapl               value_type;
      typedef std::vector<value_type> vector_type;

      for (std::size_t i = 0; i < N; ++i) {
        promise<vector_type> p;
        future<vector_type> f = p.get_future();
        async_rmi(right, this->get_rmi_handle(),
                  &p_test::vector_return<value_type>, i, std::move(p));
        const vector_type v = f.get();
        STAPL_RUNTIME_TEST_CHECK(check_vector(v, i), true);
      }
    }

    rmi_fence(); // quiescence before next test
  }

  void test_vector_deferred(void)
  {
    const std::size_t N = 100;

    {
      typedef int                     value_type;
      typedef std::vector<value_type> vector_type;

      future<vector_type> f[N];
      for (std::size_t i = 0; i < N; ++i) {
        promise<vector_type> p;
        f[i] = p.get_future();
        async_rmi(right, this->get_rmi_handle(),
                  &p_test::vector_return<value_type>, i, std::move(p));
      }

      for (std::size_t i = 0; i < N; ++i) {
        f[i].wait();
      }

      for (std::size_t i = 0; i < N; ++i) {
        const vector_type v = f[i].get();
        STAPL_RUNTIME_TEST_CHECK(check_vector(v, i), true);
      }
    }

    {
      typedef pod_stapl               value_type;
      typedef std::vector<value_type> vector_type;

      future<vector_type> f[N];
      for (std::size_t i = 0; i < N; ++i) {
        promise<vector_type> p;
        f[i] = p.get_future();
        async_rmi(right, this->get_rmi_handle(),
                  &p_test::vector_return<value_type>, i, std::move(p));
      }

      for (std::size_t i = 0; i < N; ++i) {
        f[i].wait();
      }

      for (std::size_t i = 0; i < N; ++i) {
        const vector_type v = f[i].get();
        STAPL_RUNTIME_TEST_CHECK(check_vector(v, i), true);
      }
    }

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_fundamental();
    test_vector();
    test_vector_deferred();
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

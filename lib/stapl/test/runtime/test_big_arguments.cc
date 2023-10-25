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
/// Test transporting big objects.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <algorithm>
#include <vector>
#include "test_utils.h"

using namespace stapl;

template<std::size_t Size>
struct big_object
{
  static const std::size_t size = Size;

  char *data;

  big_object(void)
  : data(new char[size])
  {
    for (std::size_t i = 0; i<size; ++i)
      data[i] = char(i);
  }

  big_object(big_object const& other)
  : data(new char[size])
  {
    std::copy(other.data, other.data + size, data);
    STAPL_RUNTIME_TEST_CHECK(test(), true);
  }

  ~big_object(void)
  { delete[] data; }

  big_object& operator=(big_object const&) = delete;

  bool test(void) const
  {
    for (std::size_t i = 0; i<size; ++i) {
      if (data[i] != char(i))
        return false;
    }
    return true;
  }

  void define_type(typer& t)
  { t.member(data, size); }
};


class test_helper
: public p_test_object
{
public:
  template<typename T>
  void test_object(T t)
  { STAPL_RUNTIME_TEST_CHECK(t.test(), true); }

  template<typename T>
  void test_object_cr(T const& t)
  { STAPL_RUNTIME_TEST_CHECK(t.test(), true); }

  template<typename T>
  void test_object_r(T& t)
  { STAPL_RUNTIME_TEST_CHECK(t.test(), true); }

  template<typename T>
  T get_object(T const& t)
  {
    STAPL_RUNTIME_TEST_CHECK(t.test(), true);
    return t;
  }

  template<typename T>
  void test_stl_container(T t)
  {
    typedef typename T::const_iterator iterator_type;
    for (iterator_type it = t.begin(); it!=t.end(); ++it) {
      STAPL_RUNTIME_TEST_CHECK(*it, 42);
    }
  }

  template<typename T>
  void test_stl_container_cr(T const& t)
  {
    typedef typename T::const_iterator iterator_type;
    for (iterator_type it = t.begin(); it!=t.end(); ++it) {
      STAPL_RUNTIME_TEST_CHECK(*it, 42);
    }
  }

  template<typename T>
  void test_stl_container_r(T& t)
  {
    typedef typename T::const_iterator iterator_type;
    for (iterator_type it = t.begin(); it!=t.end(); ++it) {
      STAPL_RUNTIME_TEST_CHECK(*it, 42);
    }
  }

  template<typename T>
  T get_stl_container(T const& t)
  {
    test_stl_container(t);
    return t;
  }
};

static const std::size_t SIZE = 100000;
static const int         N    = 5;

class p_test
: public p_object
{
public:
  void test_async_rmi(void)
  {
    typedef big_object<SIZE> arg_type;
    arg_type o;
    test_helper t;

    for (int i=0; i<N; ++i) {
      async_rmi(t.get_left_neighbor(), t.get_rmi_handle(),
                &test_helper::test_object<arg_type>, o);
    }
    for (int i=0; i<N; ++i) {
      async_rmi(t.get_left_neighbor(), t.get_rmi_handle(),
                &test_helper::test_object_cr<arg_type>, o);
    }
    for (int i=0; i<N; ++i) {
      async_rmi(t.get_left_neighbor(), t.get_rmi_handle(),
                &test_helper::test_object_r<arg_type>, o);
    }
    for (int i=0; i<N; ++i) {
      async_rmi(t.get_right_neighbor(), t.get_rmi_handle(),
                &test_helper::test_object<arg_type>, o);
      async_rmi(t.get_right_neighbor(), t.get_rmi_handle(),
                &test_helper::test_object_cr<arg_type>, o);
      async_rmi(t.get_right_neighbor(), t.get_rmi_handle(),
                &test_helper::test_object_r<arg_type>, o);
    }

    rmi_fence(); // quiescence before next test
  }

  void test_sync_rmi(void)
  {
    typedef big_object<SIZE> arg_type;
    arg_type o;
    test_helper t;
    t.test_object_cr(o);

    rmi_fence(); // wait for data to be set on all locations

    for (int i=0; i<N; ++i) {
      async_rmi(t.get_left_neighbor(), t.get_rmi_handle(),
                &test_helper::test_object<arg_type>, o);
    }
    for (int i=0; i<N; ++i) {
      async_rmi(t.get_left_neighbor(), t.get_rmi_handle(),
                &test_helper::test_object_cr<arg_type>, o);
    }
    for (int i=0; i<N; ++i) {
      async_rmi(t.get_left_neighbor(), t.get_rmi_handle(),
                &test_helper::test_object_r<arg_type>, o);
    }
    for (int i=0; i<N; ++i) {
      arg_type ot = sync_rmi(t.get_right_neighbor(), t.get_rmi_handle(),
                             &test_helper::get_object<arg_type>, o);
      async_rmi(t.get_right_neighbor(), t.get_rmi_handle(),
                &test_helper::test_object_cr<arg_type>, o);
      t.test_object_cr(ot);
    }
    for (int i=0; i<N; ++i) {
      async_rmi(t.get_right_neighbor(), t.get_rmi_handle(),
                &test_helper::test_object_cr<arg_type>, o);
    }

    rmi_fence(); // quiescence before next test
  }

  void test_vector(void)
  {
    typedef std::vector<int> arg_type;
    arg_type o(SIZE, 42);
    test_helper t;
    t.test_stl_container(o);

    rmi_fence(); // wait for data to be set on all locations

    for (int i=0; i<N; ++i) {
      async_rmi(t.get_left_neighbor(), t.get_rmi_handle(),
                &test_helper::test_stl_container<arg_type>, o);
      async_rmi(t.get_left_neighbor(), t.get_rmi_handle(),
                &test_helper::test_stl_container_r<arg_type>, o);
      async_rmi(t.get_left_neighbor(), t.get_rmi_handle(),
                &test_helper::test_stl_container_cr<arg_type>, o);
      arg_type ot = sync_rmi(t.get_right_neighbor(), t.get_rmi_handle(),
                              &test_helper::get_stl_container<arg_type>, o);
      t.test_stl_container(ot);
      async_rmi(t.get_right_neighbor(), t.get_rmi_handle(),
                &test_helper::test_stl_container_cr<arg_type>, ot);
    }

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_async_rmi();
    test_sync_rmi();
    test_vector();
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

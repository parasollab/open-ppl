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
/// Test @ref stapl::broadcast_rmi().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <algorithm>
#include <iostream>
#include <utility>
#include <vector>
#include "test_utils.h"

using namespace stapl;

class p_test
: public p_object
{
private:
  p_test_object m_obj;

public:
  void f(void) const
  { }

  // test with empty function
  void test_empty(void)
  {
    for (unsigned int i = 0; i < this->get_num_locations(); ++i) {
      const bool root = (get_location_id()==i);
      if (root)
        broadcast_rmi(root_location, m_obj.get_rmi_handle(), &p_test::f).get();
      else
        broadcast_rmi(i, &p_test::f).get();
    }

    rmi_fence(); // quiescence before next test
  }

  // test with identity member function
  void test_identity(void)
  {
    typedef std::pair<unsigned int, unsigned int> pair_type;

    for (unsigned int i = 0; i < this->get_num_locations(); ++i) {
      const bool root = (get_location_id()==i);
      const pair_type p(i, i+1);
      pair_type r;
      if (root)
        r = broadcast_rmi(root_location, m_obj.get_rmi_handle(),
                          &p_test_object::get_arg<pair_type>, p).get();
      else
        r = broadcast_rmi(i, &p_test_object::get_arg<pair_type>).get();
      STAPL_RUNTIME_TEST_CHECK(r, p);
    }

    rmi_fence(); // quiescence before next test
  }

  // test with int
  void test_int(void)
  {
    for (unsigned int i = 0; i < this->get_num_locations(); ++i) {
      const bool root = (get_location_id()==i);
      if (root)
        m_obj.set(0, int(i));
      int r = 0;
      if (root)
        r = broadcast_rmi(root_location, m_obj.get_rmi_handle(),
                          &p_test_object::get, 0).get();
      else
        r = broadcast_rmi(i, &p_test_object::get).get();
      STAPL_RUNTIME_TEST_CHECK(r, int(i));
    }

    rmi_fence(); // quiescence before next test
  }

  // test with vector<int>
  void test_vector(void)
  {
    typedef std::vector<int> vector_type;

    const vector_type v(this->get_location_id(), 42);
    for (unsigned int i = 0; i < this->get_num_locations(); ++i) {
      const bool root = (get_location_id()==i);
      vector_type r;
      if (root)
        r = broadcast_rmi(root_location, m_obj.get_rmi_handle(),
                          &p_test_object::get_arg<vector_type>, v).get();
      else
        r = broadcast_rmi(i, &p_test_object::get_arg<vector_type>).get();

      const vector_type test_v(i, 42);
      STAPL_RUNTIME_TEST_CHECK(std::equal(r.begin(), r.end(), test_v.begin()),
                               true);
    }

    rmi_fence(); // quiescence before next test
  }

  // test with vector<vector<double>>
  void test_vector_vector(void)
  {
    typedef std::vector<std::vector<double>> vector_type;

    const std::size_t N_OUTER = 1000;
    const std::size_t N_INNER = 3;

    const vector_type v(N_OUTER, std::vector<double>(N_INNER, 2.0));

    for (unsigned int i = 0; i < this->get_num_locations(); ++i) {
      const bool root = (get_location_id()==i);
      future<vector_type> f;
      if (root)
        f = broadcast_rmi(root_location, m_obj.get_rmi_handle(),
                          &p_test_object::get_arg<vector_type>, v);
      else
        f = broadcast_rmi(i, &p_test_object::get_arg<vector_type>);

      const vector_type r = f.get();
      STAPL_RUNTIME_TEST_CHECK(r.size(), N_OUTER);
      for (auto const& k : r) {
        STAPL_RUNTIME_TEST_CHECK(k.size(), N_INNER);
        for (auto l : k)
          STAPL_RUNTIME_TEST_CHECK(l, 2.0);
      }
    }

    rmi_fence(); // quiescence before next test
  }

  // test with requests made before broadcast_rmi()
  void test_rmi_before(void)
  {
    for (unsigned int i = 0; i < this->get_num_locations(); ++i) {
      const bool root = (get_location_id()==i);
      if (root)
        async_rmi(i, m_obj.get_rmi_handle(), &p_test_object::set, 0, i);
      int r = 0;
      if (root)
        r = broadcast_rmi(root_location, m_obj.get_rmi_handle(),
                          &p_test_object::get, 0).get();
      else
        r = broadcast_rmi(i, &p_test_object::get).get();
      STAPL_RUNTIME_TEST_CHECK(r, int(i));
    }

    rmi_fence(); // quiescence before next test
  }

  // test with request made after broadcast_rmi()
  void test_rmi_after(void)
  {
    for (unsigned int i = 0; i < this->get_num_locations(); ++i) {
      const bool root = (get_location_id()==i);
      if (root)
        m_obj.set(0, int(i));
      future<int> f;
      if (root) {
        f = broadcast_rmi(root_location, m_obj.get_rmi_handle(),
                          &p_test_object::get, 0);
        async_rmi(i, m_obj.get_rmi_handle(), &p_test_object::set, 0, (i+1));
      }
      else {
        f = broadcast_rmi(i, &p_test_object::get);
      }
      STAPL_RUNTIME_TEST_CHECK(f.get(), int(i));
    }

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_empty();
    test_identity();
    test_int();
    test_vector();
    test_vector_vector();
    test_rmi_before();
    test_rmi_after();
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

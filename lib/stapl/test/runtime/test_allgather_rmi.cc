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
/// Test @ref stapl::allgather_rmi().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <vector>
#include "test_utils.h"

using namespace stapl;

class p_test
: public p_object
{
private:
  p_test_object m_obj;

public:
  // test allgather_rmi() with a delay on one location
  void test_delay(void)
  {
    if (get_location_id()==0) {
      delay(1);
    }

    // test regular iteration
    typedef p_object::size_type data_type;
    futures<data_type> h1 =
      allgather_rmi(m_obj.get_rmi_handle(), &p_test_object::get_location_id);

    STAPL_RUNTIME_TEST_CHECK(m_obj.get_num_locations(), h1.size());
    for (futures<data_type>::size_type i = 0; i<h1.size(); ++i) {
      STAPL_RUNTIME_TEST_CHECK(i, h1.get(i));
    }

    rmi_fence(); // quiescence before next test
  }

  // test allgather_rmi() for int
  void test_int(void)
  {
    typedef p_object::size_type data_type;

    // test regular iteration
    futures<data_type> h1 =
      allgather_rmi(m_obj.get_rmi_handle(), &p_test_object::get_location_id);

    STAPL_RUNTIME_TEST_CHECK(m_obj.get_num_locations(), h1.size());
    for (futures<data_type>::size_type i = 0; i<h1.size(); ++i) {
      STAPL_RUNTIME_TEST_CHECK(i, h1.get(i));
    }

    rmi_fence(); // quiescence before next test

    // test vector return
    futures<data_type> h2 = allgather_rmi(m_obj.get_rmi_handle(),
                                          &p_test_object::get_arg<data_type>,
                                          m_obj.get_location_id());

    STAPL_RUNTIME_TEST_CHECK(m_obj.get_num_locations(), h2.size());
    std::vector<data_type> v2 = h2.get();
    STAPL_RUNTIME_TEST_CHECK(m_obj.get_num_locations(), v2.size());
    for (std::vector<data_type>::size_type i = 0; i<v2.size(); ++i) {
      STAPL_RUNTIME_TEST_CHECK(i, v2[i]);
    }

    rmi_fence(); // quiescence before next test
  }

  // test allgather_rmi() for std::vector<double>
  void test_vector(void)
  {
    typedef std::vector<std::vector<double>>::size_type int_type;

    // test regular iteration
    futures<std::vector<double> > h1 =
      allgather_rmi(m_obj.get_rmi_handle(), &p_test_object::get_vector<double>);

    STAPL_RUNTIME_TEST_CHECK(this->get_num_locations(), h1.size());
    for (int_type i = 0; i<h1.size(); ++i) {
      std::vector<double> v = h1.get(i);
      STAPL_RUNTIME_TEST_CHECK(i, v.size());
      for (std::vector<double>::const_iterator it=v.begin();
             it!=v.end(); ++it) {
        STAPL_RUNTIME_TEST_CHECK(i, int_type(*it));
      }
    }

    rmi_fence(); // quiescence before next test

    // test vector return
    futures<std::vector<double> > h2 =
     allgather_rmi(m_obj.get_rmi_handle(), &p_test_object::get_vector<double>);

    STAPL_RUNTIME_TEST_CHECK(this->get_num_locations(), h2.size());
    std::vector<std::vector<double> > v2 = h2.get();
    STAPL_RUNTIME_TEST_CHECK(this->get_num_locations(), v2.size());
    for (int_type i = 0; i<v2.size(); ++i) {
      std::vector<double>& v = v2[i];
      STAPL_RUNTIME_TEST_CHECK(i, v.size());
      for (std::vector<double>::const_iterator it=v.begin();
             it!=v.end(); ++it) {
        STAPL_RUNTIME_TEST_CHECK(i, int_type(*it));
      }
    }

    rmi_fence(); // quiescence before next test
  }

  // test with requests made before allgather_rmi
  void test_rmi_before(void)
  {
    for (unsigned int i = 0; i < this->get_num_locations(); ++i) {
      // set values
      async_rmi(this->get_location_id(), m_obj.get_rmi_handle(),
                &p_test_object::set, 0, i);

      // get values back
      futures<int> h =
        allgather_rmi(m_obj.get_rmi_handle(), &p_test_object::get, 0);

      std::vector<int> v = h.get();
      for (std::vector<int>::const_iterator it=v.begin(); it!=v.end(); ++it) {
        STAPL_RUNTIME_TEST_CHECK(int(i), *it);
      }
      rmi_fence(); // quiescence before next iteration
    }
  }

  // test with requests made after allgather_rmi
  void test_rmi_after(void)
  {
    for (unsigned int i = 0; i < this->get_num_locations(); ++i) {
      // set values
      m_obj.reset();
      m_obj.set(0, i);
      rmi_fence(); // wait for set() to finish

      // get values back
      futures<int> h =
        allgather_rmi(m_obj.get_rmi_handle(), &p_test_object::get, 0);

      // set new values
      async_rmi(this->get_location_id(), m_obj.get_rmi_handle(),
                &p_test_object::set, 0, 0);

      std::vector<int> v = h.get();
      for (std::vector<int>::const_iterator it=v.begin(); it!=v.end(); ++it) {
        STAPL_RUNTIME_TEST_CHECK(int(i), *it);
      }
    }
    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_delay();
    test_int();
    test_vector();
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

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
/// Test @ref stapl::opaque_rmi() to all locations.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <vector>
#include "test_utils.h"

using namespace stapl;

struct p_test
: public p_object
{
  p_test_object m_obj;

  void test_void(void)
  {
    typedef void data_type;
    // test regular iteration
    futures<data_type> h1 = opaque_rmi(all_locations, m_obj.get_rmi_handle(),
                                       &p_test_object::set_sync,
                                       m_obj.get_location_id());
    STAPL_RUNTIME_TEST_CHECK(m_obj.get_num_locations(), h1.size());
    for (futures<data_type>::size_type i = 0; i<h1.size(); ++i) {
      h1.get(i);
    }

    rmi_fence(); // wait for all opaque_rmi calls to finish

    for (unsigned int i = 0; i < m_obj.get_num_locations(); ++i) {
      STAPL_RUNTIME_TEST_CHECK(true, m_obj.get_sync(i));
    }

    rmi_fence(); // quiescence before next test

    // test vector return
    futures<data_type> h2 = opaque_rmi(all_locations, m_obj.get_rmi_handle(),
                                       &p_test_object::set_sync,
                                       m_obj.get_location_id());
    STAPL_RUNTIME_TEST_CHECK(m_obj.get_num_locations(), h2.size());
    h2.get();

    rmi_fence(); // wait for all opaque_rmi calls to finish

    for (unsigned int i = 0; i < m_obj.get_num_locations(); ++i) {
      STAPL_RUNTIME_TEST_CHECK(true, m_obj.get_sync(i));
    }

    rmi_fence(); // quiescence before next test
  }

  void test_int(void)
  {
    typedef p_object::size_type data_type;
    // test regular iteration
    futures<data_type> h1 = opaque_rmi(all_locations, m_obj.get_rmi_handle(),
                                       &p_test_object::get_location_id);
    STAPL_RUNTIME_TEST_CHECK(m_obj.get_num_locations(), h1.size());
    for (futures<data_type>::size_type i = 0; i<h1.size(); ++i) {
      STAPL_RUNTIME_TEST_CHECK(i, h1.get(i));
    }

    rmi_fence(); // quiescence before next test

    // test vector return
    futures<data_type> h2 = opaque_rmi(all_locations, m_obj.get_rmi_handle(),
                                       &p_test_object::get_arg<data_type>,
                                       m_obj.get_location_id());
    STAPL_RUNTIME_TEST_CHECK(m_obj.get_num_locations(), h2.size());
    std::vector<data_type> v2 = h2.get();
    STAPL_RUNTIME_TEST_CHECK(m_obj.get_num_locations(), v2.size());
    for (std::vector<data_type>::size_type i = 0; i<v2.size(); ++i) {
      STAPL_RUNTIME_TEST_CHECK(m_obj.get_location_id(), v2[i]);
    }

    rmi_fence(); // quiescence before next test
  }

  void test_vector(void)
  {
    typedef std::vector<std::vector<double> >::size_type int_type;

    // test regular iteration
    futures<std::vector<double> > h1 =
      opaque_rmi(all_locations, m_obj.get_rmi_handle(),
                 &p_test_object::get_vector<double>);
    STAPL_RUNTIME_TEST_CHECK(this->get_num_locations(), h1.size());
    for (int_type i = 0; i<h1.size(); ++i) {
      std::vector<double> v = h1.get(i);
      STAPL_RUNTIME_TEST_CHECK(i, v.size());
      for (std::vector<double>::const_iterator it=v.begin(); it!=v.end(); ++it)
        STAPL_RUNTIME_TEST_CHECK(i, int_type(*it));
    }

    rmi_fence(); // quiescence before next test

    // test vector return
    futures<std::vector<double> > h2 =
      opaque_rmi(all_locations, m_obj.get_rmi_handle(),
                 &p_test_object::get_vector<double>);
    STAPL_RUNTIME_TEST_CHECK(this->get_num_locations(), h2.size());
    std::vector<std::vector<double> > v2 = h2.get();
    STAPL_RUNTIME_TEST_CHECK(this->get_num_locations(), v2.size());
    for (int_type i = 0; i<v2.size(); ++i) {
      std::vector<double>& v = v2[i];
      STAPL_RUNTIME_TEST_CHECK(i, v.size());
      for (std::vector<double>::const_iterator it=v.begin(); it!=v.end(); ++it)
        STAPL_RUNTIME_TEST_CHECK(i, int_type(*it));
    }

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_void();
    test_int();
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

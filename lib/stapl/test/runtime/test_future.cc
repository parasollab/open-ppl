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
/// Test @ref stapl::future and continuation support.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
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

  template<typename T>
  T identity(const T t)
  { return t; }

  void test_then_int(void)
  {
    const int t = 10;
    int i = 0;
    future<int> fi = opaque_rmi(m_right, this->get_rmi_handle(),
                                &p_test::identity<int>, t);
    fi.async_then([&](future<int> f) {
              STAPL_RUNTIME_TEST_REQUIRE(f.valid());
              i = f.get();
            });

    rmi_fence(); // wait for all locations to finish before checking
    STAPL_RUNTIME_TEST_CHECK(i, t);
  }

  void test_then_vector(void)
  {
    const std::vector<int> t = { 1, 2, 3, 4, 5, 6 };
    std::vector<int> i;
    {
      future<std::vector<int>> fi =
        opaque_rmi(m_right, this->get_rmi_handle(),
                   &p_test::identity<std::vector<int>>, t);
      fi.async_then([&](future<std::vector<int>> f) {
                STAPL_RUNTIME_TEST_REQUIRE(f.valid());
                i = f.get();
              });
    }

    rmi_fence(); // wait for all locations to finish before checking
    STAPL_RUNTIME_TEST_RANGE(i, t);
  }

  void test_then_then(void)
  {
    const std::vector<int> t = { 1, 2, 3, 4, 5, 6 };
    std::vector<int> i;
    {
      auto pmf = &p_test::identity<std::vector<int>>;
      future<std::vector<int>> fi =
        opaque_rmi(m_right, this->get_rmi_handle(), pmf, t);

      fi.async_then([&, this](future<std::vector<int>> f) {
                STAPL_RUNTIME_TEST_REQUIRE(f.valid());
                f = opaque_rmi(this->m_right, this->get_rmi_handle(),
                               pmf, f.get());
                f.async_then([&](future<std::vector<int>> f) {
                         STAPL_RUNTIME_TEST_REQUIRE(f.valid());
                         i = f.get();
                       });
              });
    }

    rmi_fence(); // wait for all locations to finish before checking
    STAPL_RUNTIME_TEST_RANGE(i, t);
  }

  void test_futures(void)
  {
    auto pmf = &p_test::identity<unsigned int>;
    futures<unsigned int> f = opaque_rmi(all_locations, this->get_rmi_handle(),
                                         pmf, this->get_location_id());
    f.async_then([this](futures<unsigned int> f) {
             STAPL_RUNTIME_TEST_REQUIRE(f.valid());
             std::vector<unsigned int> v = f.get();
             for (auto&& i : v)
               STAPL_RUNTIME_TEST_CHECK(i, this->get_location_id());
           });

    rmi_fence(); // quiescence before next test
  }

  void test_wait_then(void)
  {
    const std::vector<int> t = { 1, 2, 3, 4, 5, 6 };
    std::vector<int> i;
    {
      future<std::vector<int>> fi =
        opaque_rmi(m_right, this->get_rmi_handle(),
                   &p_test::identity<std::vector<int>>, t);
      fi.wait();
      STAPL_RUNTIME_TEST_CHECK(fi.valid(), true);
      fi.async_then([&](future<std::vector<int>> f) {
                STAPL_RUNTIME_TEST_REQUIRE(f.valid());
                i = f.get();
              });
    }

    rmi_fence(); // wait for all locations to finish before checking
    STAPL_RUNTIME_TEST_RANGE(i, t);
  }

  void test_ready_future(void)
  {
    const std::vector<int> t = { 1, 2, 3, 4, 5, 6 };
    future<std::vector<int>> f = make_ready_future(t);
    STAPL_RUNTIME_TEST_CHECK(f.valid(), true);
    auto v = f.get();
    STAPL_RUNTIME_TEST_RANGE(v, t);
  }

  void test_ready_futures(void)
  {
    const std::vector<int> t = { 1, 2, 3, 4, 5, 6 };
    futures<int> f = make_ready_futures(std::begin(t), std::end(t));
    STAPL_RUNTIME_TEST_CHECK(f.valid(), true);
    auto v = f.get();
    STAPL_RUNTIME_TEST_RANGE(v, t);
  }

  void execute(void)
  {
    test_then_int();
    test_then_vector();
    test_then_then();
    test_futures();
    test_wait_then();
    test_ready_future();
    test_ready_futures();
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

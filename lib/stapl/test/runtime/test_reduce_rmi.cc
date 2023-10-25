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
/// Test @ref stapl::reduce_rmi() and @ref stapl::unordered::reduce_rmi().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <functional>
#include <algorithm>
#include <numeric>
#include <valarray>
#include "test_utils.h"

using namespace stapl;

class p_test
: public p_object
{
public:
  std::valarray<int> get_valarray(void) const
  { return std::valarray<int>(1, this->get_num_locations()); }

  // tests sync_reduce_rmi() with std::plus<> and int
  void test_add_int_b(void)
  {
    typedef std::plus<unsigned int> op_type;

    unsigned int N = 0;
    for (unsigned int s = 1; s < this->get_num_locations(); ++s) {
      N = op_type()(N, s);
    }

    const auto result = sync_reduce_rmi(op_type(), this->get_rmi_handle(),
                                        &p_test::get_location_id);
    STAPL_RUNTIME_TEST_CHECK(result, N);

    rmi_fence(); // quiescence before next test
  }

  // tests reduce_rmi() with std::plus<> and int
  void test_add_int_nb(void)
  {
    typedef std::plus<unsigned int> op_type;

    unsigned int N = 0;
    for (unsigned int s = 1; s<this->get_num_locations(); ++s) {
      N = op_type()(N, s);
    }

    future<unsigned int> result = reduce_rmi(op_type(), this->get_rmi_handle(),
                                             &p_test::get_location_id);
    STAPL_RUNTIME_TEST_CHECK(result.get(), N);

    rmi_fence(); // quiescence before next test
  }

  // tests sync_reduce_rmi() with std::plus<> and std::valarray<int>
  void test_add_valarray_b(void)
  {
    typedef std::plus<std::valarray<int>> op_type;

    const std::valarray<int> N(this->get_num_locations(),
                               this->get_num_locations());

    const std::valarray<int> result =
      sync_reduce_rmi(op_type(), this->get_rmi_handle(), &p_test::get_valarray);
    STAPL_RUNTIME_TEST_CHECK(result.sum(), N.sum());

    rmi_fence(); // quiescence before next test
  }

  // tests reduce_rmi() with std::plus<> and std::valarray<int>
  void test_add_valarray_nb(void)
  {
    typedef std::plus<std::valarray<int>> op_type;

    const std::valarray<int> N(this->get_num_locations(),
                               this->get_num_locations());

    future<std::valarray<int> > result =
      reduce_rmi(op_type(), this->get_rmi_handle(), &p_test::get_valarray);
    STAPL_RUNTIME_TEST_CHECK(result.get().sum(), N.sum());

    rmi_fence(); // quiescence before next test
  }

  // tests unordered::sync_reduce_rmi()
  void test_unordered_b(void)
  {
    typedef std::plus<unsigned int> plus_op;

    unsigned int N = 0;
    for (unsigned int s = 1; s<this->get_num_locations(); ++s) {
      N = plus_op()(N, s);
    }

    const unsigned int result =
      unordered::sync_reduce_rmi(plus_op(), this->get_rmi_handle(),
                                 &p_test::get_location_id);
    STAPL_RUNTIME_TEST_CHECK(result, N);

    rmi_fence(); // quiescence before next test
  }

  // tests unordered::reduce_rmi()
  void test_unordered_nb(void)
  {
    typedef std::plus<unsigned int> plus_op;

    unsigned int N = 0;
    for (unsigned int s = 1; s<this->get_num_locations(); ++s) {
      N = plus_op()(N, s);
    }

    future<unsigned int> result =
      unordered::reduce_rmi(plus_op(), this->get_rmi_handle(),
                            &p_test::get_location_id);
    result.wait();
    STAPL_RUNTIME_TEST_CHECK(result.get(), N);

    rmi_fence(); // quiescence before next test
  }

  void execute()
  {
    test_add_int_b();
    test_add_int_nb();

    test_add_valarray_b();
    test_add_valarray_nb();

    test_unordered_b();
    test_unordered_nb();
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

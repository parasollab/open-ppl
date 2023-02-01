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
/// Test @ref stapl::allreduce_rmi().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <algorithm>
#include <functional>
#include <vector>
#include "test_utils.h"

using namespace stapl;

template<typename T>
struct add_vector
{
  std::vector<T> operator()(std::vector<T> const& v1,
                            std::vector<T> const& v2) const
  {
    std::vector<T> v{v1};
    v.insert(v.end(), v2.begin(), v2.end());
    return v;
  }
};


class p_test
: public p_object
{
public:
  unsigned int get_unique_number(const unsigned int n) const
  { return (this->get_location_id() + n); }

  // test for commutative operator
  void test_commutative(void)
  {
    typedef unsigned int          value_type;
    typedef std::plus<value_type> operator_type;

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, false);

    operator_type op;

    value_type N = 0;
    for (value_type i = 0; i < this->get_num_locations(); ++i) {
      N = op(N, i);
    }

    for (unsigned int i = 0; i < 100; ++i) {
      future<value_type> r = allreduce_rmi(op, this->get_rmi_handle(),
                                           &p_test::get_unique_number, i);
      value_type result = r.get();
      STAPL_RUNTIME_TEST_CHECK(result, (N + i*get_num_locations()));
    }

    rmi_fence(); // quiescence before next test
  }

  std::vector<int> get_vector(int i)
  { return std::vector<int>(1, i); }

  // test for non-commutative operator
  void test_non_commutative(void)
  {
    typedef std::vector<int>                                value_type;
    typedef add_vector<int>                                 base_operator_type;
    typedef decltype(non_commutative(base_operator_type{})) operator_type;

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, true);

    operator_type op{base_operator_type{}};

    value_type N;
    for (unsigned int i = 0; i < this->get_num_locations(); ++i) {
      N.push_back(i);
    }

    for (int i = 0; i < 100; ++i) {
      future<value_type> r =
        allreduce_rmi(op, this->get_rmi_handle(), &p_test::get_vector,
                      this->get_location_id());
      value_type result = r.get();
      STAPL_RUNTIME_TEST_CHECK(N.size(), result.size());
      STAPL_RUNTIME_TEST_CHECK(std::equal(N.begin(), N.end(), result.begin()),
                               true);
    }

    rmi_fence(); // quiescence before next test
  }

  // test for commutative operator with continuation
  void test_commutative_async(void)
  {
    typedef unsigned int          value_type;
    typedef std::plus<value_type> operator_type;

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, false);

    operator_type op;

    value_type N = 0;
    for (value_type i = 0; i < this->get_num_locations(); ++i) {
      N = op(N, i);
    }

    for (int i = 0; i < 2; ++i) {
      future<value_type> f = allreduce_rmi(op, this->get_rmi_handle(),
                                           &p_test::get_unique_number, i);

      value_type result = 0;
      bool done         = false;
      f.async_then([&](future<value_type> f)
                   {
                     result = f.get();
                     done   = true;
                   });
      block_until([&done] { return done; });
      STAPL_RUNTIME_TEST_CHECK(result, (N + i*get_num_locations()));
    }

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_commutative();
    test_non_commutative();
    test_commutative_async();
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

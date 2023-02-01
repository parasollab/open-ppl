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
/// Test @ref stapl::runtime::allreduce_object.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <stapl/runtime/collective/allreduce_object.hpp>
#include <stapl/runtime/type_traits/is_non_commutative.hpp>
#include <algorithm>
#include <deque>
#include <functional>
#include <iostream>
#include "test_utils.h"

using namespace stapl;

struct A
{
  int m_x;
};

template <typename T>
struct B
{ };

template <typename T>
struct C
{
  T operator()(T const& lhs, T const& rhs)
  {
    return lhs + rhs;
  }
};

using basic_type                           = double;
using non_basic_type                       = A;
template <typename T> using empty_op       = B<T>;
template <typename T> using commutative_op = std::plus<T>;
template <typename T> using non_commute_op =
  decltype(non_commutative(C<T>()));

static_assert(std::is_same<
      stapl::runtime::allreduce_object<
        basic_type,
        commutative_op<basic_type>>,
      stapl::runtime::allreduce_object<
        basic_type,
        commutative_op<basic_type>,
        false,
        false>>::value,
    "Incorrect allreduce_object specialization deduction");

static_assert(std::is_same<
      stapl::runtime::allreduce_object<
        non_basic_type,
        commutative_op<non_basic_type>>,
      stapl::runtime::allreduce_object<
        non_basic_type,
        commutative_op<non_basic_type>,
        false,
        true>>::value,
    "Incorrect allreduce_object specialization deduction");

static_assert(std::is_same<
      stapl::runtime::allreduce_object<
        basic_type,
        non_commute_op<basic_type>>,
      stapl::runtime::allreduce_object<
        basic_type,
        non_commute_op<basic_type>,
        true,
        false>>::value,
    "Incorrect allreduce_object specialization deduction");

static_assert(std::is_same<
      stapl::runtime::allreduce_object<
        non_basic_type,
        non_commute_op<non_basic_type>>,
      stapl::runtime::allreduce_object<
        non_basic_type,
        non_commute_op<non_basic_type>,
        true,
        true>>::value,
    "Incorrect allreduce_object specialization deduction");

static_assert(std::is_same<
      stapl::runtime::allreduce_object<
        basic_type,
        empty_op<basic_type>>,
      stapl::runtime::allreduce_object<
        basic_type,
        empty_op<basic_type>,
        false,
        false>>::value,
    "Incorrect allreduce_object specialization deduction");

static_assert(std::is_same<
      stapl::runtime::allreduce_object<
        non_basic_type,
        empty_op<non_basic_type>>,
      stapl::runtime::allreduce_object<
        non_basic_type,
        empty_op<non_basic_type>,
        false,
        true>>::value,
    "Incorrect allreduce_object specialization deduction");

class p_test
: public p_object
{
public:
  // test for commutative operator
  void test_commutative(void)
  {
    using namespace runtime;

    using value_type    = unsigned int;
    using operator_type = std::plus<value_type>;

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, false);

    operator_type op;

    value_type N = 0;
    for (value_type i = 0; i < this->get_num_locations(); ++i) {
      N = op(N, i);
    }

    allreduce_object<value_type, operator_type> r{this_context::get(), op};

    for (unsigned int i = 0; i < 100; ++i) {
      r(this->get_location_id() + i);
      value_type result = r.get();
      STAPL_RUNTIME_TEST_CHECK(result, (N + i*get_num_locations()));
    }

    rmi_fence(); // quiescence before next test
  }

  // test for commutative operator with continuation
  void test_commutative_async(void)
  {
    using namespace runtime;

    using value_type    = unsigned int;
    using operator_type = std::plus<value_type>;

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, false);

    operator_type op;

    value_type N = 0;
    for (value_type i = 0; i < this->get_num_locations(); ++i) {
      N = op(N, i);
    }

    allreduce_object<value_type, operator_type> r{this_context::get(), op};

    for (unsigned int i = 0; i < 100; ++i) {
      r(this->get_location_id() + i);

      value_type result = 0;
      bool done         = false;
      r.async_then([&](future<value_type> f)
                   {
                     result = f.get();
                     done   = true;
                   });
      block_until([&done] { return done; });
      STAPL_RUNTIME_TEST_CHECK(result, (N + i*get_num_locations()));
    }

    rmi_fence(); // quiescence before next test
  }

  // test a lot of allreduce_objects fired in ordered fashion using commutative
  // operations
  void test_commutative_ordered(void)
  {
    using namespace runtime;

    using value_type    = unsigned int;
    using operator_type = std::plus<value_type>;

    const std::size_t NUM   = 100;
    const std::size_t ITERS = 1000;

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, false);

    operator_type op;

    value_type N = 0;
    for (value_type i = 0; i < this->get_num_locations(); ++i) {
      N = op(N, i);
    }

    std::deque<allreduce_object<value_type, operator_type>> d;

    for (auto i = 0u; i < NUM; ++i) {
      d.emplace_back(std::ref(this_context::get()), op);
    }

    for (auto i = 0u; i < ITERS; ++i) {
      for (auto j = 0u; j < NUM; ++j)
        d[j](this->get_location_id() + j);

      for (auto j = 0u; j < NUM; ++j) {
        value_type result = d[j].get();
        STAPL_RUNTIME_TEST_CHECK(result, (N + j*get_num_locations()));
      }
    }

    rmi_fence(); // quiescence before next test
  }

  // test a lot of allreduce_objects fired in ordered fashion with async_then()
  // using commutative operations
  void test_commutative_ordered_async(void)
  {
    using namespace runtime;

    using value_type    = unsigned int;
    using operator_type = std::plus<value_type>;

    const std::size_t NUM   = 100;
    const std::size_t ITERS = 1000;

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, false);

    operator_type op;

    value_type N = 0;
    for (value_type i = 0; i < this->get_num_locations(); ++i) {
      N = op(N, i);
    }

    std::deque<allreduce_object<value_type, operator_type>> d;

    for (auto i = 0u; i < NUM; ++i) {
      d.emplace_back(std::ref(this_context::get()), op);
    }

    for (auto i = 0u; i < ITERS; ++i) {
      unsigned int pending = NUM;

      for (auto j = 0u; j < NUM; ++j) {
        d[j](this->get_location_id() + j);
        const auto res = (N + j*get_num_locations());
        d[j].async_then(
          [res, &pending](future<value_type> f)
          {
            STAPL_RUNTIME_TEST_CHECK(f.get(), res);
            --pending;
          });
      }

      block_until([&pending] { return (pending==0); });
    }

    rmi_fence(); // quiescence before next test
  }

  // test for non-commutative operator
  void test_non_comm_basic(void)
  {
    using namespace runtime;

    using value_type    = unsigned int;
    using base_op_type  = C<value_type>;
    using operator_type = decltype(non_commutative(base_op_type{}));

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, true);

    operator_type op{base_op_type{}};

    value_type N = 0;

    for (value_type i = 0; i < this->get_num_locations(); ++i)
      N = op(N, i);

    for (unsigned int i = 0; i < 100; ++i)
    {
      allreduce_object<value_type,
                       operator_type,
                       is_non_commutative<operator_type>::value,
                       true
                      > r{this_context::get(), op};

      r(this->get_location_id() + i);
      value_type result = r.get();
      STAPL_RUNTIME_TEST_CHECK(result, (N + i*get_num_locations()));
    }

    rmi_fence(); // quiescence before next test
  }

  // test for non-commutative operator with continuation
  void test_non_comm_async_basic(void)
  {
    using namespace runtime;

    using value_type    = unsigned int;
    using base_op_type  = C<value_type>;
    using operator_type = decltype(non_commutative(base_op_type{}));

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, true);

    operator_type op{base_op_type{}};

    value_type N = 0;

    for (value_type i = 0; i < this->get_num_locations(); ++i)
      N = op(N, i);

    for (unsigned int i = 0; i < 100; ++i)
    {
      allreduce_object<value_type,
                       operator_type,
                       is_non_commutative<operator_type>::value,
                       true
                      > r{this_context::get(), op};

      r(this->get_location_id() + i);

      value_type result = 0;
      bool done         = false;
      r.async_then([&](future<value_type> f)
                   {
                     result = f.get();
                     done   = true;
                   });
      block_until([&done] { return done; });
      STAPL_RUNTIME_TEST_CHECK(result, (N + i*get_num_locations()));
    }

    rmi_fence(); // quiescence before next test
  }

  // test a lot of allreduce_objects fired in ordered fashion using
  // non-commutative operations
  void test_non_comm_ordered_basic(void)
  {
    using namespace runtime;

    using value_type    = unsigned int;
    using base_op_type  = C<value_type>;
    using operator_type = decltype(non_commutative(base_op_type{}));

    const std::size_t NUM   = 100;
    const std::size_t ITERS = 1000;

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, true);

    operator_type op{base_op_type{}};

    value_type N = 0;

    for (value_type i = 0; i < this->get_num_locations(); ++i)
      N = op(N, i);

    for (auto i = 0u; i < ITERS; ++i)
    {
      std::deque<allreduce_object<value_type,
                                  operator_type,
                                  is_non_commutative<operator_type>::value,
                                  true
                                 >> d;

      for (auto i = 0u; i < NUM; ++i)
        d.emplace_back(std::ref(this_context::get()), op);

      for (auto j = 0u; j < NUM; ++j)
        d[j](this->get_location_id() + j);

      for (auto j = 0u; j < NUM; ++j) {
        value_type result = d[j].get();
        STAPL_RUNTIME_TEST_CHECK(result, (N + j*get_num_locations()));
      }
    }

    rmi_fence(); // quiescence before next test
  }

  // test a lot of allreduce_objects fired in ordered fashion with async_then()
  // and using non-commutative operations
  void test_non_comm_ordered_async_basic(void)
  {
    using namespace runtime;

    using value_type    = unsigned int;
    using base_op_type  = C<value_type>;
    using operator_type = decltype(non_commutative(base_op_type{}));

    const std::size_t NUM   = 100;
    const std::size_t ITERS = 1000;

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, true);

    operator_type op{base_op_type{}};

    value_type N = 0;

    for (value_type i = 0; i < this->get_num_locations(); ++i)
      N = op(N, i);

    for (auto i = 0u; i < ITERS; ++i)
    {
      std::deque<allreduce_object<value_type,
                                  operator_type,
                                  is_non_commutative<operator_type>::value,
                                  true
                                 >> d;

      for (auto i = 0u; i < NUM; ++i)
        d.emplace_back(std::ref(this_context::get()), op);

      unsigned int pending = NUM;

      for (auto j = 0u; j < NUM; ++j)
      {
        d[j](this->get_location_id() + j);
        const auto res = (N + j*get_num_locations());
        d[j].async_then(
          [res, &pending](future<value_type> f)
          {
            STAPL_RUNTIME_TEST_CHECK(f.get(), res);
            --pending;
          });
      }

      block_until([&pending] { return (pending==0); });
    }

    rmi_fence(); // quiescence before next test
  }

  // test for non-commutative operator
  void test_non_comm_non_basic(void)
  {
    using namespace runtime;

    using value_type    = unsigned int;
    using base_op_type  = C<value_type>;
    using operator_type = decltype(non_commutative(base_op_type{}));

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, true);

    operator_type op{base_op_type{}};

    value_type N = 0;

    for (value_type i = 0; i < this->get_num_locations(); ++i)
      N = op(N, i);

    for (unsigned int i = 0; i < 100; ++i)
    {
      allreduce_object<value_type,
                       operator_type,
                       is_non_commutative<operator_type>::value,
                       false
                      > r{this_context::get(), op};

      r(this->get_location_id() + i);
      value_type result = r.get();
      STAPL_RUNTIME_TEST_CHECK(result, (N + i*get_num_locations()));
    }

    rmi_fence(); // quiescence before next test
  }

  // test for non-commutative operator with continuation
  void test_non_comm_async_non_basic(void)
  {
    using namespace runtime;

    using value_type    = unsigned int;
    using base_op_type  = C<value_type>;
    using operator_type = decltype(non_commutative(base_op_type{}));

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, true);

    operator_type op{base_op_type{}};

    value_type N = 0;

    for (value_type i = 0; i < this->get_num_locations(); ++i)
      N = op(N, i);

    for (unsigned int i = 0; i < 100; ++i)
    {
      allreduce_object<value_type,
                       operator_type,
                       is_non_commutative<operator_type>::value,
                       false
                      > r{this_context::get(), op};

      r(this->get_location_id() + i);

      value_type result = 0;
      bool done         = false;
      r.async_then([&](future<value_type> f)
                   {
                     result = f.get();
                     done   = true;
                   });
      block_until([&done] { return done; });
      STAPL_RUNTIME_TEST_CHECK(result, (N + i*get_num_locations()));
    }

    rmi_fence(); // quiescence before next test
  }

  // test a lot of allreduce_objects fired in ordered fashion using
  // non-commutative operations
  void test_non_comm_ordered_non_basic(void)
  {
    using namespace runtime;

    using value_type    = unsigned int;
    using base_op_type  = C<value_type>;
    using operator_type = decltype(non_commutative(base_op_type{}));

    const std::size_t NUM   = 100;
    const std::size_t ITERS = 1000;

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, true);

    operator_type op{base_op_type{}};

    value_type N = 0;

    for (value_type i = 0; i < this->get_num_locations(); ++i)
      N = op(N, i);

    for (auto i = 0u; i < ITERS; ++i)
    {
      std::deque<allreduce_object<value_type,
                                  operator_type,
                                  is_non_commutative<operator_type>::value,
                                  false
                                 >> d;

      for (auto i = 0u; i < NUM; ++i)
        d.emplace_back(std::ref(this_context::get()), op);

      for (auto j = 0u; j < NUM; ++j)
        d[j](this->get_location_id() + j);

      for (auto j = 0u; j < NUM; ++j) {
        value_type result = d[j].get();
        STAPL_RUNTIME_TEST_CHECK(result, (N + j*get_num_locations()));
      }
    }

    rmi_fence(); // quiescence before next test
  }

  // test a lot of allreduce_objects fired in ordered fashion with async_then()
  // and using non-commutative operations
  void test_non_comm_ordered_async_non_basic(void)
  {
    using namespace runtime;

    using value_type    = unsigned int;
    using base_op_type  = C<value_type>;
    using operator_type = decltype(non_commutative(base_op_type{}));

    const std::size_t NUM   = 100;
    const std::size_t ITERS = 1000;

    STAPL_RUNTIME_TEST_CHECK(is_non_commutative<operator_type>::value, true);

    operator_type op{base_op_type{}};

    value_type N = 0;

    for (value_type i = 0; i < this->get_num_locations(); ++i)
      N = op(N, i);

    for (auto i = 0u; i < ITERS; ++i)
    {
      std::deque<allreduce_object<value_type,
                                  operator_type,
                                  is_non_commutative<operator_type>::value,
                                  false
                                 >> d;

      for (auto i = 0u; i < NUM; ++i)
        d.emplace_back(std::ref(this_context::get()), op);

      unsigned int pending = NUM;

      for (auto j = 0u; j < NUM; ++j)
      {
        d[j](this->get_location_id() + j);
        const auto res = (N + j*get_num_locations());
        d[j].async_then(
          [res, &pending](future<value_type> f)
          {
            STAPL_RUNTIME_TEST_CHECK(f.get(), res);
            --pending;
          });
      }

      block_until([&pending] { return (pending==0); });
    }

    rmi_fence(); // quiescence before next test
  }

  void execute(void)
  {
    test_commutative();
    test_commutative_async();
    test_commutative_ordered();
    test_commutative_ordered_async();

    test_non_comm_non_basic();
    test_non_comm_async_non_basic();
    test_non_comm_ordered_non_basic();
    test_non_comm_ordered_async_non_basic();

    test_non_comm_basic();
    test_non_comm_async_basic();
    test_non_comm_ordered_basic();
    test_non_comm_ordered_async_basic();
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

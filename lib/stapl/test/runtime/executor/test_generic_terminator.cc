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
/// Test @ref stapl::terminator.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <stapl/runtime/executor/terminator.hpp>
#include <iostream>
#include <algorithm>
#include <functional>
#include "../test_utils.h"

using namespace stapl;

const unsigned int MAX_PENDING = 1000;
const unsigned int ITERATIONS  = 100;

class p_test
: public p_object
{
private:
  unsigned int m_right;  // right neighbor
  unsigned int m_pending;

public:
  p_test(void)
  : m_right(0),
    m_pending(MAX_PENDING)
  {
    const auto id = this->get_location_id();
    m_right       = (id==this->get_num_locations()-1) ? 0 : (id + 1);
  }

public:
  void reduce_pending(void)
  {
    if (m_pending>0)
      --m_pending;
  }

  unsigned int pending(void)
  {
    // RMI required to exit the block_until loop
    async_rmi(m_right, this->get_rmi_handle(), &p_test::reduce_pending);
    return m_pending;
  }

  struct term_reduce
  {
    using value_type = std::pair<unsigned int, bool>;

    value_type operator()(value_type const& lhs, value_type const& rhs) const
    {
      return value_type(lhs.first + rhs.first, lhs.second && rhs.second);
    }
  };

  // tests one terminator
  void test_single_terminator(void)
  {
    using pair_t = std::pair<unsigned int, bool>;

    using terminator_type = terminator<pair_t, term_reduce>;

    terminator_type t{term_reduce{},
                      [this] { return pair_t(this->pending(), false); }};

    bool done = false;
    m_pending = MAX_PENDING;

    t.set_notifier([&done]
                   {
                     STAPL_RUNTIME_TEST_CHECK(done, false);
                     done = true;
                   });

    for (auto i = 0u; i < ITERATIONS; ++i) {
      t();
      block_until([&done] { return done; });

      STAPL_RUNTIME_TEST_CHECK(done, true);
      STAPL_RUNTIME_TEST_CHECK(m_pending, 0);
      done      = false;
      m_pending = MAX_PENDING;

      rmi_fence(); // quiescence before next iteration
    }
  }

  // test a lot of terminators fired in ordered fashion
  void test_ordered_terminators(void)
  {
    using pair_t = std::pair<unsigned int, bool>;

    using terminator_type = terminator<pair_t, term_reduce>;

    const std::size_t NUM_TERMINATORS = 1000;

    std::deque<terminator_type> terminators;

    unsigned int pending = NUM_TERMINATORS;

    for (auto i = 0u; i < NUM_TERMINATORS; ++i) {
      terminators.emplace_back(term_reduce{}, [] { return pair_t(0, false); });
      auto& t = terminators.back();
      t.set_notifier([&pending]
                     {
                       STAPL_RUNTIME_TEST_REQUIRE(pending > 0);
                       --pending;
                     });
    }

    for (auto i = 0u; i < ITERATIONS; ++i) {
      for (auto j = 0u; j < NUM_TERMINATORS; ++j)
        terminators[j]();

      block_until([&pending] { return (pending==0); });

      STAPL_RUNTIME_TEST_CHECK(pending, 0);
      pending = NUM_TERMINATORS;
    }
    rmi_fence(); // quiescence before next iteration
  }

  void execute(void)
  {
    test_single_terminator();
    test_ordered_terminators();
  }
};


exit_code stapl_main(int, char*[])
{
  // test was not made to work with one location
  if (get_num_locations()==1)
    return EXIT_SUCCESS;

  p_test pt;
  pt.execute();
#ifndef _TEST_QUIET
  std::cout << get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}

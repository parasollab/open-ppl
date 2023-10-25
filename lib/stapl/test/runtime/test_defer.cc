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
/// Test RI deferral mechanism to ensure atomicity with respect to
/// local computation (aka "local" rmi).
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include <random>
#include "test_utils.h"

using namespace stapl;

enum class lock_behavior
{
  no_lock, lock_guard, try_lock
};

template<lock_behavior LockBehavior>
class obj
 : public p_object
{
private:
  bool m_flag;
  bool m_b_failure;

  template<typename F>
  void work_impl(F&& f)
  {
    m_flag = true;
    f();

    for (location_type i = 0; i < this->get_num_locations(); ++i)
      if (i != this->get_location_id())
        async_rmi(i, this->get_rmi_handle(), &obj::unset_flag);

     if (!m_flag)
       m_b_failure = true;
  }

public:
  obj(void)
    : m_flag(false),
      m_b_failure(false)
  { this->advance_epoch(); }

  template<typename F>
  void work(F&& f)
  {
    switch (LockBehavior)
    {
      case lock_behavior::lock_guard: {
        std::lock_guard<p_object> lg{*this};
        work_impl(std::forward<F>(f));
        break;
      }

      case lock_behavior::try_lock: {
        STAPL_RUNTIME_TEST_REQUIRE(this->try_lock());
        work_impl(std::forward<F>(f));
        this->unlock();
        break;
      }

      case lock_behavior::no_lock: {
        work_impl(std::forward<F>(f));
        break;
      }
    }
  }

  void unset_flag()
  { m_flag = false; }

  bool check_failure() const
  { return m_b_failure; }
};


template<lock_behavior LockBehavior>
void test_defer(const int max_size, const std::size_t trials)
{
  // Can use std::random_device{}(), but prefer repeatability.
  std::mt19937 gen(get_location_id());
  std::uniform_int_distribution<> dis{0, max_size};
  double x = 2.321232;

  for (std::size_t i = 0; i < trials; ++i) {
    obj<LockBehavior> o;
    o.work([&]() {
      std::size_t num_iters = dis(gen);

      for (std::size_t i = 0; i < num_iters; ++i)
        x = std::hypot(x, std::sin(x));
    });

    rmi_fence();

    STAPL_RUNTIME_TEST_REQUIRE(!o.check_failure());
  }
}

exit_code stapl_main(int argc, char* argv[])
{
  const int max_size        = std::atol(argv[1]);
  const std::size_t trials  = std::atol(argv[2]);

  test_defer<lock_behavior::no_lock>(max_size, trials);
  test_defer<lock_behavior::lock_guard>(max_size, trials);
  test_defer<lock_behavior::try_lock>(max_size, trials);

  return EXIT_SUCCESS;
}

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE timer
#include "utility.h"
#include <stapl/runtime/counter/default_counters.hpp>
#include <stapl/runtime/utility/timer.hpp>

using namespace stapl;

#ifdef STAPL_USE_PAPI
# include <stapl/runtime/utility/papi_cycle_clock.hpp>

using timer_type = runtime::timer<runtime::papi_cycle_clock>;

static int initialize(void)
{
  if (PAPI_is_initialized()!=PAPI_LOW_LEVEL_INITED &&
      PAPI_library_init(PAPI_VER_CURRENT)!=PAPI_VER_CURRENT)
    std::abort();
  return 0;
}

int init = initialize();

#else

using timer_type = runtime::timer<>;

#endif


BOOST_AUTO_TEST_CASE( test_timer_remaining )
{
  counter<default_timer> c;

  const unsigned long timeout = 200;
  timer_type t{std::chrono::milliseconds(timeout)};
  t.reset();

  c.start();
  while (t.remaining() > std::chrono::milliseconds::zero());
  c.stop();

  const double time = c.value() * 1000.0;
  const double lower_bound = timeout * 0.9;
  const double upper_bound = timeout * 1.1;
  BOOST_CHECK( (lower_bound<time) && (time<upper_bound) );
}


BOOST_AUTO_TEST_CASE( test_timer_expired )
{
  counter<default_timer> c;

  const unsigned long timeout = 200;
  timer_type t{std::chrono::milliseconds(timeout)};
  t.reset();

  c.start();
  auto tt = t.now();
  while (!t.expired(tt))
    tt = t.now();
  c.stop();

  const double time = c.value()*1000.0;
  const double lower_bound = timeout * 0.9;
  const double upper_bound = timeout * 1.1;
  BOOST_CHECK( (lower_bound<time) && (time<upper_bound) );
}

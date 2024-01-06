/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE papi_timers
#include <chrono>
#include <thread>
#include "utility.h"
#include <stapl/runtime/counter/counter.hpp>
#include <stapl/runtime/counter/papi/papi_counter.hpp>
#include <stapl/runtime/counter/papi/papi_cycle_counter.hpp>
#include <stapl/runtime/counter/papi/papi_timer_n_counter.hpp>
#include <stapl/runtime/counter/papi/papi_timer.hpp>
#include <cmath>

using namespace stapl;

static int initialize(void)
{
  if (PAPI_is_initialized()!=PAPI_LOW_LEVEL_INITED &&
      PAPI_library_init(PAPI_VER_CURRENT)!=PAPI_VER_CURRENT)
    std::abort();
  return 0;
}

int init = initialize();

static void loop(void)
{
  std::this_thread::sleep_for(std::chrono::seconds{1});
}


BOOST_AUTO_TEST_CASE( test_papi_counter )
{
  counter<papi_counter> c(PAPI_L1_DCM, PAPI_TOT_CYC);
  std::cout << c.native_name() << ": ";

  c.start();
  loop();
  counter<papi_counter>::value_type v = c.stop();

  typedef counter<papi_counter>::value_type::const_iterator iterator_type;
  for (iterator_type begin = v.begin(), end = v.end(); begin!=end; ++begin) {
    std::cout << *begin << ' ';
  }
  std::cout << std::endl;
}

BOOST_AUTO_TEST_CASE( test_papi_cycle_counter )
{
  counter<papi_cycle_counter> c;
  std::cout << c.native_name() << ": " << c.read() << ' ';

  c.start();
  loop();
  std::cout << c.stop() << std::endl;
}

BOOST_AUTO_TEST_CASE( test_papi_timer_n_counter )
{
  counter<papi_timer_n_counter> c(PAPI_L1_DCM);
  std::cout << c.native_name() << ": ";

  c.start();
  loop();
  counter<papi_timer_n_counter>::value_type v = c.stop();

  std::cout << v.time << "  ";
  typedef counter<papi_counter>::value_type::const_iterator iterator_type;
  for (iterator_type begin = v.counters.begin(), end = v.counters.end();
         begin!=end; ++begin) {
    std::cout << *begin << ' ';
  }
  std::cout << std::endl;
}

BOOST_AUTO_TEST_CASE( test_papi_timer )
{
  counter<papi_timer> c;
  std::cout << c.native_name() << ": " << c.read() << ' ';

  c.start();
  loop();
  std::cout << c.stop() << std::endl;
}

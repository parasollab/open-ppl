/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE posix_timers
#include "utility.h"
#include <chrono>
#include <thread>
#include <stapl/runtime/counter/counter.hpp>
#include <stapl/runtime/counter/posix/clock_gettime_timer.hpp>
#include <stapl/runtime/counter/posix/getrusage_timer.hpp>
#include <stapl/runtime/counter/posix/gettimeofday_timer.hpp>
#include <cmath>

using namespace stapl;

static void loop(void)
{
  std::this_thread::sleep_for(std::chrono::seconds{1});
}

BOOST_AUTO_TEST_CASE( test_clock_gettime_timer )
{
  counter<clock_gettime_timer> c;
  std::cout << c.native_name() << ": " << c.read() << ' ';

  c.start();
  loop();
  std::cout << c.stop() << std::endl;
}

BOOST_AUTO_TEST_CASE( test_getrusage_timer )
{
  counter<getrusage_timer> c;
  std::cout << c.native_name() << ": " << std::endl;

  c.start();
  loop();
  counter<getrusage_timer>::value_type v = c.stop();
  std::cout << "\tru_utime.tv_sec:  " << v.ru_utime.tv_sec << std::endl;
  std::cout << "\tru_utime.tv_usec: " << v.ru_utime.tv_usec << std::endl;
  std::cout << "\tru_stime.tv_sec:  " << v.ru_stime.tv_sec << std::endl;
  std::cout << "\tru_stime.tv_usec: " << v.ru_stime.tv_usec << std::endl;
  std::cout << "\tru_maxrss:        " << v.ru_maxrss << std::endl;
  std::cout << "\tru_ixrss:         " << v.ru_ixrss << std::endl;
  std::cout << "\tru_idrss:         " << v.ru_idrss << std::endl;
  std::cout << "\tru_isrss:         " << v.ru_isrss << std::endl;
  std::cout << "\tru_minflt:        " << v.ru_minflt << std::endl;
  std::cout << "\tru_majflt:        " << v.ru_majflt << std::endl;
  std::cout << "\tru_nswap:         " << v.ru_nswap << std::endl;
  std::cout << "\tru_inblock:       " << v.ru_inblock << std::endl;
  std::cout << "\tru_oublock:       " << v.ru_oublock << std::endl;
  std::cout << "\tru_msgsnd:        " << v.ru_msgsnd << std::endl;
  std::cout << "\tru_msgrcv:        " << v.ru_msgrcv << std::endl;
  std::cout << "\tru_nsignals       " << v.ru_nsignals << std::endl;
  std::cout << "\tru_nvcsw:         " << v.ru_nvcsw << std::endl;
  std::cout << "\tru_nivcsw:        " << v.ru_nivcsw << std::endl;
}

BOOST_AUTO_TEST_CASE( test_gettimeofday_timer )
{
  counter<gettimeofday_timer> c;
  std::cout << c.native_name() << ": " << c.read() << ' ';

  c.start();
  loop();
  std::cout << c.stop() << std::endl;
}

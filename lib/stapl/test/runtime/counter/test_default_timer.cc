/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE default_timer
#include "utility.h"
#include <chrono>
#include <thread>
#include <stapl/runtime/counter/default_counters.hpp>

using namespace stapl;

BOOST_AUTO_TEST_CASE( test_default_timer )
{
  counter<default_timer> c;
  std::cout << c.native_name() << ": " << c.read() << ' ';

  // test one time event
  c.start();
  std::this_thread::sleep_for(std::chrono::seconds{1});
  const counter<default_timer>::value_type time1 = c.stop();

  // test reset
  c.reset();
  c.start();
  std::this_thread::sleep_for(std::chrono::seconds{1});
  const counter<default_timer>::value_type time2 = c.stop();

  // test accumulation
  c.start();
  std::this_thread::sleep_for(std::chrono::seconds{1});
  const counter<default_timer>::value_type time3 = c.stop();

  std::cout << time1 << ' ' << time2 << ' ' << time3 << std::endl;
}

BOOST_AUTO_TEST_CASE( test_scoped_counter )
{
  counter<default_timer> c;
  std::cout << c.native_name() << ": " << c.read() << ' ';

  // test one time event
  {
    scoped_counter<counter<default_timer> > s(c);
    std::this_thread::sleep_for(std::chrono::seconds{1});
  }
  const counter<default_timer>::value_type time1 = c.value();

  // test reset
  {
    c.reset();
    scoped_counter<counter<default_timer> > s(c);
    std::this_thread::sleep_for(std::chrono::seconds{1});
  }
  const counter<default_timer>::value_type time2 = c.value();

  // test accumulation
  {
    scoped_counter<counter<default_timer> > s(c);
    std::this_thread::sleep_for(std::chrono::seconds{1});
  }
  const counter<default_timer>::value_type time3 = c.value();

  std::cout << time1 << ' ' << time2 << ' ' << time3 << std::endl;
}


namespace stapl {

template<>
struct disable_group_counter<1>
: public std::true_type
{ };

} // namespace stapl

BOOST_AUTO_TEST_CASE( test_disabled )
{
  counter<default_timer, 1> c;

  {
    scoped_counter<counter<default_timer, 1> > s(c);
    std::this_thread::sleep_for(std::chrono::seconds{1});
  }
  const counter<default_timer, 1>::value_type time = c.value();

  BOOST_CHECK_EQUAL(time, 0.0);
}

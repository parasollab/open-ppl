/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE test_probe
#include "utility.h"
#include <chrono>
#include <thread>
#include <stapl/runtime/counter/probe.hpp>
#include <stapl/runtime/counter/default_counters.hpp>

using namespace stapl;

BOOST_AUTO_TEST_CASE( test_probe_clear )
{
  {
    auto p1 = probe<counter<default_timer>>::get("first");
    auto p2 = probe<counter<default_timer>>::get("second");
    std::this_thread::sleep_for(std::chrono::seconds{1});
  }
  probe<counter<default_timer>>::clear();
}


BOOST_AUTO_TEST_CASE( test_probe_1 )
{
  auto t = std::thread([]{
    auto p1 = probe<counter<default_timer>>::get("first");
    auto p2 = probe<counter<default_timer>>::get("second");
    std::this_thread::sleep_for(std::chrono::seconds{1});
  });

  auto p1 = probe<counter<default_timer>>::get("first");
  auto p2 = probe<counter<default_timer>>::get("second");
  std::this_thread::sleep_for(std::chrono::seconds{1});

  t.join();
}


BOOST_AUTO_TEST_CASE( test_probe_dump )
{
  {
    auto p1 = probe<counter<default_timer>>::get("first");
    auto p2 = probe<counter<default_timer>>::get("second");
    std::this_thread::sleep_for(std::chrono::seconds{1});
  }
  probe<counter<default_timer>>::dump(std::cout);
}

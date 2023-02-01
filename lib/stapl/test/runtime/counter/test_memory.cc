/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE process_memory_usage
#include "utility.h"
#include <stapl/runtime/counter/counter.hpp>
#include <stapl/runtime/counter/default_counters.hpp>

using namespace stapl;


BOOST_AUTO_TEST_CASE( test_process_memory_usage_counter )
{
  const std::size_t N = 100000000;
  counter<process_memory_usage_counter> c;
  std::cout << c.native_name() << ":\n";

  c.start();
  char* a = new char[N];
  std::memset(a, '0', N * sizeof(*a));
  std::cout << "\tAfter allocation: "<< c.stop() << std::endl;

  c.restart();
  delete[] a;
  std::cout << "\tAfter free: " << c.stop() << std::endl;
}

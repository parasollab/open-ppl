/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#define STAPL_RUNTIME_TEST_MODULE test_memory
#include "utility.h"
#include <stapl/runtime/system.hpp>
#include <cstring>

using namespace stapl::runtime;

BOOST_AUTO_TEST_CASE( total_memory )
{
  const std::size_t mem = get_total_physical_memory();
  BOOST_TEST_MESSAGE("Total memory (MB): " << (mem/1024.0/1024.0));
}


BOOST_AUTO_TEST_CASE( used_memory )
{
  const std::size_t mem = get_used_physical_memory();
  BOOST_TEST_MESSAGE("Used memory (MB): " << (mem/1024.0/1024.0));
}


BOOST_AUTO_TEST_CASE( available_memory )
{
  const std::size_t mem = get_available_physical_memory();
  BOOST_TEST_MESSAGE("Available memory (MB): " << (mem/1024.0/1024.0));
}


BOOST_AUTO_TEST_CASE( comparison )
{
  BOOST_CHECK( get_total_physical_memory() > get_used_physical_memory() );
  BOOST_CHECK( get_total_physical_memory() > get_available_physical_memory() );
}


BOOST_AUTO_TEST_CASE( detect )
{
  const std::size_t consume_mb = 200; // consume this many MBs
  const std::size_t size = consume_mb * (1024*1024); // memory in bytes

  char* volatile c = new char[size];
  std::memset(c, '0', size*sizeof(*c)); // touch memory

  const std::size_t total = get_total_physical_memory();
  const std::size_t free  = get_available_physical_memory();

  BOOST_CHECK( total >= (free + size) );

  delete[] c;
}

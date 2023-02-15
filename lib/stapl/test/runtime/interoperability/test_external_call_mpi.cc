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
/// Test calling MPI functions through @ref stapl::external_call().
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <iostream>
#include "../test_utils.h"

#ifndef STAPL_DONT_USE_MPI
# include <mpi.h>
#endif

int allreduce(int i)
{
#ifndef STAPL_DONT_USE_MPI
  MPI_Allreduce(MPI_IN_PLACE, &i, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
#endif
  return i;
}

void barrier(void)
{
#ifndef STAPL_DONT_USE_MPI
  MPI_Barrier(MPI_COMM_WORLD);
#endif
}

void allreduce_p(int* i)
{
#ifndef STAPL_DONT_USE_MPI
  MPI_Allreduce(MPI_IN_PLACE, i, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
#endif
}


stapl::exit_code stapl_main(int, char*[])
{
  using namespace stapl;
  std::set<unsigned int> c = external_callers();

  boost::optional<int> p = external_call(&allreduce, 1);
  if (c.find(get_location_id())!=c.end()) {
    STAPL_RUNTIME_TEST_REQUIRE(bool(p));
    STAPL_RUNTIME_TEST_CHECK(*p, get_num_processes());
  }
  else {
    STAPL_RUNTIME_TEST_REQUIRE(!bool(p));
  }

  bool f = stapl::external_call(&barrier);
  if (c.find(get_location_id())!=c.end()) {
    STAPL_RUNTIME_TEST_REQUIRE(f);
  }
  else {
    STAPL_RUNTIME_TEST_REQUIRE(!f);
  }

  int  i = 1;
  bool b = stapl::external_call(&allreduce_p, &i);
  if (c.find(get_location_id())!=c.end()) {
    STAPL_RUNTIME_TEST_REQUIRE(b);
    STAPL_RUNTIME_TEST_CHECK(i, get_num_processes());
  }
  else {
    STAPL_RUNTIME_TEST_REQUIRE(!b);
    STAPL_RUNTIME_TEST_CHECK(i, 1);
  }

#ifndef _TEST_QUIET
  std::cout << stapl::get_location_id() << " successfully passed!" << std::endl;
#endif
  return EXIT_SUCCESS;
}

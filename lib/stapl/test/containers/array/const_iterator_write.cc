/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/array/array.hpp>
#include <stapl/containers/array/static_array.hpp>
using namespace stapl;

exit_code stapl_main(int argc, char* argv[])
{
  array<size_t> arr(5);
  for (array<size_t>::const_iterator cit = arr.cbegin();
       cit != arr.cend(); ++cit) {
    *cit = 1; //should fail to compile
  }

  array<size_t> stat_arr(5);
  for (array<size_t>::const_iterator cit = stat_arr.cbegin();
       cit != stat_arr.cend(); ++cit) {
    *cit = 1; //should fail to compile
  }

  return EXIT_SUCCESS;
}

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/vector/vector.hpp>
using namespace stapl;

exit_code stapl_main(int argc, char* argv[])
{
  vector<size_t> vec(5);
  for (vector<size_t>::const_iterator cit = vec.cbegin();
       cit != vec.cend(); ++cit) {
    *cit = 1; //should fail to compile
  }

  return EXIT_SUCCESS;
}

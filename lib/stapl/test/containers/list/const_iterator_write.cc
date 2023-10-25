/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/list/list.hpp>
using namespace stapl;

exit_code stapl_main(int argc, char* argv[])
{
  list<size_t> lis(5);
  for (list<size_t>::const_iterator cit = lis.cbegin();
       cit != lis.cend(); ++cit) {
    *cit = 1; //should fail to compile
  }

  return EXIT_SUCCESS;
}

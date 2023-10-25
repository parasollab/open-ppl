/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>

#include "../expect.hpp"

stapl::exit_code stapl_main(int, char*[])
{
  int x = 5;
  const int y = 5;
  const int z = 6;

  stapl::tests::expect_eq(x, y) << "eq";

  stapl::tests::expect_ne(x, z) << "ne";

  stapl::tests::expect_lt(x, z) << "lt";

  stapl::tests::expect_le(x, y) << "le";
  stapl::tests::expect_le(x, z) << "le";

  stapl::tests::expect_gt(z, y) << "gt";

  stapl::tests::expect_ge(z, x) << "ge";
  stapl::tests::expect_ge(z, z) << "ge";

  return EXIT_SUCCESS;
}

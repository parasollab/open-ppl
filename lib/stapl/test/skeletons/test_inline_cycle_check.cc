/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include "../test_report.hpp"

using namespace stapl;

exit_code stapl_main(int argc, char** argv)
{
  using namespace stapl::skeletons;

  using value_type = size_t;
  using op = plus<value_type>;

  DECLARE_INLINE_INPUT_PLACEHOLDERS(2, in)
  DECLARE_INLINE_PLACEHOLDERS(2, x)

  // This should fail to compile under sorted_inline_flow
  auto s = compose<tags::sorted_inline_flow>(
      x0 << zip(op{}) | (in0, in1),
      x1 << zip(op{}) | (in0, x1) // Self-edge here
  );

  return EXIT_SUCCESS;
}

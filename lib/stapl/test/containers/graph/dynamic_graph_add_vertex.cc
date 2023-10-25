/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <stapl/graph.hpp>
#include "../../test_report.hpp"

using namespace stapl;

stapl::exit_code stapl_main(int argc, char **argv)
{
  using graf_tp = stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES>;

  const std::size_t n = 100;
  const std::size_t p = get_num_locations();

  graf_tp gr_vw(n);

  gr_vw.add_vertex();

  rmi_fence();

  const std::size_t size = gr_vw.size();

  bool passed = size == n + p;

  STAPL_TEST_REPORT(passed, "Testing vertex descriptor generator");

  return EXIT_SUCCESS;
}

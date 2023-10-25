/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/generators/binary_tree.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include <test/algorithms/test_utils.h>
#include "../../test_report.hpp"


using namespace stapl;
using namespace std;

template<typename Graph>
void test_has_edge(void)
{
  const size_t num_levels = 8;
  const size_t expected_size = pow(2, num_levels)-1;

  using view_type = graph_view<Graph>;

  // generate vertices and edges in graph
  auto v = generators::make_binary_tree<view_type>(expected_size, false);

  auto f0 = v.has_edge(0,1);
  auto f1 = v.has_edge(0,3);

  stapl::stapl_bool b(f0.get() && !f1.get());
  bool passed = b.reduce();

  STAPL_TEST_REPORT(passed, "Binary tree has_edge");
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  test_has_edge<dynamic_graph<DIRECTED, MULTIEDGES>>();
  test_has_edge<graph<DIRECTED, MULTIEDGES>>();

  return EXIT_SUCCESS;
}

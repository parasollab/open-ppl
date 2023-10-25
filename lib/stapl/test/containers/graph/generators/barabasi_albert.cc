/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/generators/barabasi_albert.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>

#include "../../../test_report.hpp"

using namespace stapl;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cout << "usage: " << argv[0] << " n d" << std::endl;
    exit(1);
  }

  const std::size_t n = atol(argv[1]);
  const std::size_t d = atol(argv[2]);

  // types for graph and generator
  using graph_type = graph<DIRECTED, MULTIEDGES>;
  using view_type = graph_view<graph_type>;

  auto v = generators::make_barabasi_albert<view_type>(n, d);

  STAPL_TEST_REPORT(v.size() == n, "Testing size of Barabasi-Albert");
  STAPL_TEST_REPORT(v.num_edges() == 2*n*d,
    "Testing number of edges of Barabasi-Albert");

  return EXIT_SUCCESS;
}

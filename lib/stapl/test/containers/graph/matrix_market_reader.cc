/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "test_util.h"
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/algorithms/algorithm.hpp>

using namespace stapl;

exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe edge-list matrix-market" << std::endl;
    exit(1);
  }

  std::string el_filename = argv[1];
  std::string mm_filename = argv[2];

  using graph_type = graph<DIRECTED, MULTIEDGES>;

  auto el = read_edge_list<graph_type>(el_filename);
  auto mm = read_matrix_market<graph_type>(mm_filename);

  one_print(compare_graphs(el, mm));

  return EXIT_SUCCESS;
}


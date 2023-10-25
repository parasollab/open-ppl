/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "test_util.h"
#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/algorithms/algorithm.hpp>

using namespace stapl;

exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cerr << "usage: exe filename metadata_filename" << std::endl;
    exit(1);
  }

  std::string filename = argv[1];
  std::string metadata_filename = argv[2];

  using graph_type = graph<DIRECTED, MULTIEDGES>;

  auto adj = read_adj_list<graph_type>(filename);
  auto sharded = sharded_graph_reader<graph_type>(
    metadata_filename, read_adj_list_line()
  );

  one_print(compare_graphs(adj, sharded));

  return EXIT_SUCCESS;
}


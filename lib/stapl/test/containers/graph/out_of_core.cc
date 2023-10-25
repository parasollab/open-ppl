/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/containers/graph/out_of_core_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/array/array.hpp>
#include <boost/lexical_cast.hpp>
#include <stapl/runtime/counter/default_counters.hpp>

#include <vector>
#include <algorithm>

#include "test_util.h"

using namespace stapl;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t n  = 128;
  size_t blk_sz  = 32;

  if (argc > 1) {
    n = atoi(argv[1]);
  }

  one_print("Testing Out-of-Core Graph Construction...\t\t");
  out_of_core_graph<DIRECTED, MULTIEDGES, int, int> g(n, blk_sz);
  one_print(g.size() == n);

  return EXIT_SUCCESS;
}

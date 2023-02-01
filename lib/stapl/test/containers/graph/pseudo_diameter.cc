/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/pseudo_diameter.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>

#include "test_util.h"

using namespace std;

stapl::exit_code stapl_main(int argc,char** argv)
{
  size_t nx, ny;
  size_t tuning = 0;

  if (argc > 2) {
    nx = atol(argv[1]);
    ny = atol(argv[2]);
  } else {
    cout<<"usage: exe x-dim y-dim\n";
    return EXIT_FAILURE;
  }

  for (int i = 1; i < argc; i++) {
    if (!strcmp("--tuning", argv[i]))
      tuning = atoi(argv[i+1]);
  }

  typedef stapl::multidigraph<stapl::properties::bfs_property> PGR;
  typedef stapl::graph_view<PGR> graph_view_t;
  graph_view_t vw = stapl::generators::make_mesh<graph_view_t>(nx, ny);


  // compute the diameter.
  const size_t diameter = nx + ny - 2;


  // compute the pseudo-diameter.
  one_print("Testing Pseudo Diameter...\t\t\t");
  auto exec_policy = stapl::sgl::make_execution_policy("kla", vw, tuning);
  size_t pseudo_diameter = stapl::pseudo_diameter(exec_policy, vw);
  one_print(pseudo_diameter == diameter);

  // compute the pseudo-diameter (h).
  one_print("Testing Pseudo Diameter (H)...\t\t\t");
  auto h_exec_policy = stapl::sgl::make_execution_policy("hier", vw);
  pseudo_diameter = stapl::pseudo_diameter(h_exec_policy, vw);
  one_print(pseudo_diameter == diameter);

  return EXIT_SUCCESS;
}

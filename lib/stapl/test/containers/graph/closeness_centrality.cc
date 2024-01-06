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
#include <stapl/containers/graph/algorithms/closeness_centrality.hpp>
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

  typedef stapl::multidigraph<stapl::properties::closeness_property> PGR;
  typedef stapl::graph_view<PGR> graph_view_t;
  graph_view_t vw = stapl::generators::make_mesh<graph_view_t>(nx, ny);
  size_t n = vw.size();
  size_t t = n/2;
  if (nx % 2 != 0 && ny % 2 != 0)
    ++t;

  // compute the closeness-centrality.
  one_print("Testing Closeness Centrality...\t\t\t");
  auto exec_policy = stapl::sgl::make_execution_policy("kla", vw, tuning);
  stapl::closeness_centrality(exec_policy, vw);
  one_print(
    vw[0].property().closeness() == vw[n-1].property().closeness() &&
    vw[n/2-1].property().closeness() == vw[t].property().closeness() &&
    vw[0].property().closeness() != 0);

  // compute the closeness-centrality (h).
  one_print("Testing Closeness Centrality (H)...\t\t");
  auto h_exec_policy = stapl::sgl::make_execution_policy("hier", vw);
  stapl::closeness_centrality(h_exec_policy, vw);
  one_print(
    vw[0].property().closeness() == vw[n-1].property().closeness() &&
    vw[n/2-1].property().closeness() == vw[t].property().closeness() &&
    vw[0].property().closeness() != 0);

  one_print("Testing Closeness Centrality (Hubs)...\t\t");
  auto hubs_exec_policy = stapl::sgl::make_execution_policy("hubs", vw, 3);
  stapl::closeness_centrality(hubs_exec_policy, vw);
  one_print(
    vw[0].property().closeness() == vw[n-1].property().closeness() &&
    vw[n/2-1].property().closeness() == vw[t].property().closeness() &&
    vw[0].property().closeness() != 0);

  return EXIT_SUCCESS;
}

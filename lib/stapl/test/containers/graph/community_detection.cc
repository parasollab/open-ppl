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
#include <stapl/containers/graph/algorithms/community_detection.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/containers/graph/algorithms/execution_policy.hpp>

#include "test_util.h"

using namespace std;

template<typename GraphView>
GraphView create_graph(size_t nx, size_t ny)
{
  GraphView vw = stapl::generators::make_mesh<GraphView>(nx, ny);

  stapl::do_once([&](void) {
      for (size_t i=0; i<nx*ny; ++i) {
        vw.add_edge(0, i);
        vw.add_edge(i, 0);
      }
    });

  stapl::rmi_fence(); // for add_edge to finish.

  return vw;
}


template<typename GraphView>
void test_cd(size_t nx, size_t ny)
{
  GraphView vw = create_graph<GraphView>(nx, ny);
  const size_t iter = vw.size();

  // compute the community-labels.
  one_print("Testing Community Detection...\t\t\t");
  stapl::community_detection(stapl::sgl::make_execution_policy("lsync", vw),
      vw, iter);
  one_print(vw[0].property().label() == vw[vw.size()-1].property().label());
  stapl::rmi_fence(); // for destroying the view.

}


template<typename GraphView>
void test_cd_hier(size_t nx, size_t ny)
{
  GraphView vw = create_graph<GraphView>(nx, ny);
  const size_t iter = vw.size();

  // compute the community-labels (h).
  one_print("Testing Community Detection (H)...\t\t\t");
  stapl::community_detection(stapl::sgl::make_execution_policy("hier", vw),
      vw, iter);
  one_print(vw[0].property().label() == vw[vw.size()-1].property().label());
  stapl::rmi_fence(); // for destroying the view.

}


template<typename GraphView>
void test_cd_hubs(size_t nx, size_t ny)
{
  GraphView vw = create_graph<GraphView>(nx, ny);
  const size_t iter = vw.size();

  // compute the community-labels (h-hubs).
  one_print("Testing Community Detection (H-Hubs)...\t\t\t");
  stapl::community_detection(stapl::sgl::make_execution_policy("hubs", vw, 3),
      vw, iter);
  one_print(vw[0].property().label() == vw[vw.size()-1].property().label());
  stapl::rmi_fence(); // for destroying the view.

}


stapl::exit_code stapl_main(int argc,char** argv)
{
  size_t nx, ny;

  if (argc > 2) {
    nx = atol(argv[1]);
    ny = atol(argv[2]);
  } else {
    cout<<"usage: exe x-dim y-dim\n";
    return EXIT_FAILURE;
  }

  typedef stapl::multidigraph<stapl::properties::cd_property> PGR;
  typedef stapl::graph_view<PGR> graph_view_t;

  test_cd<graph_view_t>(nx, ny);
  test_cd_hier<graph_view_t>(nx, ny);
  test_cd_hubs<graph_view_t>(nx, ny);

  return EXIT_SUCCESS;
}

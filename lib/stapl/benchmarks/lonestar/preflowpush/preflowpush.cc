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
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/containers/graph/algorithms/preflowpush.hpp>
#include <stapl/utility/do_once.hpp>
#include <cstdlib>
#include <iostream>
#include <string>


//////////////////////////////////////////////////////////////////////
/// @brief Functor to set the capacities of each edge in the graph
/// currently set up to set each edge capacity in a random number
/// between 0 and 15.
//////////////////////////////////////////////////////////////////////
struct set_capacity
{
  typedef void result_type;

  template <typename Vertex>
  result_type operator()(Vertex v)
  {
    auto adje = v.begin();
    auto ee = v.end();
    for (; adje != ee; ++adje) {
      srand((*adje).target()+v.descriptor());
      (*adje).property().rc((rand()%15)+1);
    }
  }
};

stapl::exit_code stapl_main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cout <<"usage: " << argv[0] << " x y async" << std::endl;
    exit(1);
  }

  const size_t x = atoi(argv[1]);
  const size_t y = atoi(argv[2]);
  const size_t async = atoi(argv[3]);

  const size_t n = x*y;


  typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
    stapl::properties::preflowpush_vertex_property,
    stapl::properties::preflowpush_edge_property> graph_type;

  typedef stapl::graph_view<graph_type> view_type;

  graph_type g(n);
  view_type g_view(g);

  stapl::generators::make_mesh(g_view, x, y, false);

  stapl::map_func(set_capacity(), g_view);

  auto result = stapl::preflow_push<view_type>(g_view, 0,
                                               g_view.num_vertices()-1, async);

   stapl::do_once([&](void) {
    std::cout << "global relabel time: " << stapl::get<0>(result) << std::endl;
    std::cout << "initialization time: " << stapl::get<1>(result) << std::endl;
    std::cout << "push relabel time: " << stapl::get<2>(result)<< std::endl;
    std::cout << "Flow: " << stapl::get<3>(result) << std::endl;
  });

  return EXIT_SUCCESS;
}


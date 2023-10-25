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
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>

#include "../../../test_report.hpp"
#include "generator_utils.hpp"

using namespace stapl;
using namespace std;

template<typename GraphView>
void test_output(GraphView& v, size_t x, size_t y, std::string s)
{
  STAPL_TEST_REPORT(v.size() == x*y,
                    std::string("Mesh Generator: Testing size  " + s));

  boost::unordered_map<size_t, size_t> distribution =
    degree_distribution(v);

  bool passed = true;
  if (distribution.size() == 3) {
    passed &= (distribution[4] == ((x*y) - 2*(x+y-2)));
    passed &= (distribution[3] == (2*(x+y-2) - 4));
    passed &= (distribution[2] == 4);
  } else if (distribution.size() == 2) {
    passed &= (distribution[3] == (2*(x+y-2) - 4));
    passed &= (distribution[2] == 4);
  } else if (distribution.size() == 1) {
    passed &= (distribution[2] == 4);
  } else {
    passed = false;
  }

  STAPL_TEST_REPORT(passed, std::string("Mesh Generator: Testing degree" + s));

  rmi_fence();
}

template<typename Graph>
void test_mesh_gen(size_t x, size_t y, std::string s)
{
  typedef Graph                        graph_type;
  typedef graph_view<graph_type>       view_type;

  // create graph and view
  graph_type g1;
  rmi_fence();
  view_type v1(g1);

  graph_type g2(x*y);
  rmi_fence();
  view_type v2(g2);

  // generate vertices and edges in graph
  view_type v = generators::make_mesh<view_type>(x, y);
  v1 = generators::make_mesh<view_type>(v1, x, y);
  v2 = generators::make_mesh<view_type>(v2, x, y);

  test_output(v, x, y, s);
  test_output(v1, x, y, s);
  test_output(v2, x, y, s);

  rmi_fence();
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cout << "usage: " << argv[0] << " x_dim y_dim" << std::endl;
    exit(1);
  }

  // inputs
  const size_t x = atol(argv[1]);
  const size_t y = atol(argv[2]);

  stapl_assert(x > 1 && y > 1, "validation requires non-degenerate case");

  // types for graph and generator
  typedef dynamic_graph<DIRECTED, MULTIEDGES>   d_graph_type;
  typedef dynamic_graph<UNDIRECTED, MULTIEDGES> u_graph_type;

  test_mesh_gen<d_graph_type>(x, y, " [Directed Graph]   ");
  test_mesh_gen<u_graph_type>(x, y, " [UnDirected Graph] ");

  return EXIT_SUCCESS;
}

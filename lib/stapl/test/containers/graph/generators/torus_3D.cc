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
#include <stapl/containers/graph/generators/torus_3D.hpp>

#include "../../../test_report.hpp"
#include "generator_utils.hpp"

using namespace stapl;

template<typename GraphView>
void test_output(GraphView& v, size_t x, size_t y, size_t z, std::string s)
{
  STAPL_TEST_REPORT(v.size() == x*y*z,
                    std::string("3D Torus Generator: Testing size  " + s));

  boost::unordered_map<size_t, size_t> distribution =
    degree_distribution(v);

  bool passed  = (distribution.size() == 1);
       passed &= (distribution[6] == x*y*z);

  STAPL_TEST_REPORT(passed,
                    std::string("3D Torus Generator: Testing degree" + s));

  rmi_fence();
}

template<typename Graph>
void test_3D_torus_gen(size_t x, size_t y, size_t z, std::string s)
{
  typedef Graph                        graph_type;
  typedef graph_view<graph_type>       view_type;

  // create graph and view
  graph_type g1;
  rmi_fence();
  view_type v1(g1);

  graph_type g2(x*y*z);
  rmi_fence();
  view_type v2(g2);

  // generate vertices and edges in graph
  view_type v = generators::make_torus_3D<view_type>(x, y, z);
  v1 = generators::make_torus_3D<view_type>(v1, x, y, z);
  v2 = generators::make_torus_3D<view_type>(v2, x, y, z);

  test_output(v, x, y, z, s);
  test_output(v1, x, y, z, s);
  test_output(v2, x, y, z, s);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc != 4) {
    std::cout << "usage: " << argv[0] << " x_dim y_dim z_dim" << std::endl;
    return EXIT_FAILURE;
  }

  // inputs
  const size_t x = atol(argv[1]);
  const size_t y = atol(argv[2]);
  const size_t z = atol(argv[3]);

  stapl_assert(x > 1 && y > 1 && z > 1,
               "validation requires non-degenerate case");

  // types for graph and generator
  typedef dynamic_graph<DIRECTED, MULTIEDGES>   d_graph_type;
  typedef dynamic_graph<UNDIRECTED, MULTIEDGES> u_graph_type;

  test_3D_torus_gen<d_graph_type>(x, y, z, " [Directed Graph]   ");
  test_3D_torus_gen<u_graph_type>(x, y, z, " [UnDirected Graph] ");

  return EXIT_SUCCESS;
}

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
#include <stapl/containers/graph/generators/sparse_mesh.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>

#include "../../../test_report.hpp"
#include "generator_utils.hpp"

using namespace stapl;
using namespace std;

struct degree_of
{
  using result_type = std::size_t;

  template<typename V>
  result_type operator()(V v) const
  {
    return v.size();
  }
};

template<typename GraphView>
void test_output(GraphView& v, size_t x, size_t y, std::string s)
{
  STAPL_TEST_REPORT(v.size() == x*y,
                    std::string("Sparse Mesh Generator: Testing size  " + s));

  auto max_degree = map_reduce(degree_of{}, stapl::max<std::size_t>{}, v);
  bool passed = max_degree <= 4;

  STAPL_TEST_REPORT(passed, std::string("Mesh Generator: Testing degree" + s));

  rmi_fence();
}

template<typename Graph>
void test_mesh_gen(size_t x, size_t y, double p, std::string s)
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
  view_type v = generators::make_sparse_mesh<view_type>(x, y, p);
  v1 = generators::make_sparse_mesh<view_type>(v1, x, y, p);
  v2 = generators::make_sparse_mesh<view_type>(v2, x, y, p);

  test_output(v, x, y, s);
  test_output(v1, x, y, s);
  test_output(v2, x, y, s);

  rmi_fence();
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cout << "usage: " << argv[0] << " x_dim y_dim p" << std::endl;
    exit(1);
  }

  // inputs
  const size_t x = atol(argv[1]);
  const size_t y = atol(argv[2]);
  const double p = atof(argv[3]);

  stapl_assert(x > 1 && y > 1, "validation requires non-degenerate case");

  // types for graph and generator
  typedef dynamic_graph<DIRECTED, MULTIEDGES>   d_graph_type;
  typedef dynamic_graph<UNDIRECTED, MULTIEDGES> u_graph_type;

  test_mesh_gen<d_graph_type>(x, y, p, " [Directed Graph]   ");
  test_mesh_gen<u_graph_type>(x, y, p, " [UnDirected Graph] ");

  return EXIT_SUCCESS;
}

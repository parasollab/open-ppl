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
#include <stapl/containers/graph/generators/star.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>

#include "../../../test_report.hpp"
#include "generator_utils.hpp"

using namespace stapl;
using namespace std;

template<typename GraphView>
void test_output(GraphView& v, size_t n, size_t r, std::string s)
{

  STAPL_TEST_REPORT(v.size() == n,
                    std::string("Star Generator: Testing size  " + s));

  boost::unordered_map<size_t, size_t> distribution =
    degree_distribution(v);

  // pass if there is one vertex with an edge to everyone (or no one,
  // if directed), and there are n-1 vertices with only one edge
  bool passed =
    (distribution[1] == n-1 && distribution[0] == 1) || // directed
    (distribution[1] == n-1 && distribution[n-1] == 1); // undirected

  // also check if the root is the vertex with the differing degree
  const std::size_t root_degree = v[r].size();
  passed &= root_degree == 0 || root_degree == n-1;

  STAPL_TEST_REPORT(passed, std::string("Star Generator: Testing degree" + s));

  rmi_fence();
}

template<typename Graph>
void test_star(size_t n, size_t r, std::string s)
{
  typedef Graph                        graph_type;
  typedef graph_view<graph_type>       view_type;

  // create graph and view
  graph_type g1;
  rmi_fence();
  view_type v1(g1);

  graph_type g2(n);
  rmi_fence();
  view_type v2(g2);

  // generate vertices and edges in graph
  view_type v = generators::make_star<view_type>(n, r);
  v1 = generators::make_star<view_type>(v1, n, r);
  v2 = generators::make_star<view_type>(v2, n, r);

  test_output(v, n, r, s);
  test_output(v1, n, r, s);
  test_output(v2, n, r, s);

  rmi_fence();
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cout << "usage: " << argv[0] << " x_dim y_dim" << std::endl;
    exit(1);
  }

  // inputs
  const size_t n = atol(argv[1]);
  const size_t r = atol(argv[2]);

  // types for graph and generator
  typedef dynamic_graph<DIRECTED, MULTIEDGES>   d_graph_type;
  typedef dynamic_graph<UNDIRECTED, MULTIEDGES> u_graph_type;

  test_star<d_graph_type>(n, r, " [Directed Graph]   ");
  test_star<u_graph_type>(n, r, " [UnDirected Graph] ");

  return EXIT_SUCCESS;
}

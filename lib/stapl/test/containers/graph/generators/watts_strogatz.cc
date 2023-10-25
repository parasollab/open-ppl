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
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/generators/watts_strogatz.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../../../test_report.hpp"

using namespace stapl;
using namespace std;


template<typename GraphView>
void test_output(GraphView& v, size_t n, size_t k,
                 std::string name, std::string s)
{
  STAPL_TEST_REPORT(v.size() == n,
                    "Testing size of   " + name + s);

  bool passed = v.num_edges() >= n*k/2;

  STAPL_TEST_REPORT(passed, "Testing degree of " + name + s);

  rmi_fence();
}


template<typename Graph>
void test_newman_watts_strogatz(size_t n, size_t k, double p, std::string s)
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
  view_type v =
    generators::make_newman_watts_strogatz<view_type>(n, k, p, false);
  v1 = generators::make_newman_watts_strogatz<view_type>(v1, n, k, p, false);
  v2 = generators::make_newman_watts_strogatz<view_type>(v2, n, k, p, false);

  test_output(v, n, k, "Newman-Watts-Strogatz", s);
  test_output(v1, n, k, "Newman-Watts-Strogatz", s);
  test_output(v2, n, k, "Newman-Watts-Strogatz", s);
}

template<typename Graph>
void test_watts_strogatz(size_t n, size_t k, double p, std::string s)
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
  view_type v = generators::make_watts_strogatz<view_type>(n, k, p, false);
  v1 = generators::make_watts_strogatz<view_type>(v1, n, k, p, false);
  v2 = generators::make_watts_strogatz<view_type>(v2, n, k, p, false);

  test_output(v, n, k, "       Watts-Strogatz", s);
  test_output(v1, n, k, "       Watts-Strogatz", s);
  test_output(v2, n, k, "       Watts-Strogatz", s);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 4) {
    std::cout << "usage: " << argv[0] << " n k p" << std::endl;
    exit(1);
  }

  // inputs
  const size_t n = atol(argv[1]);
  const size_t k = atol(argv[2]);
  const double p = atof(argv[3]);

  srand(0);

  // types for graph and generator
  typedef dynamic_graph<DIRECTED, MULTIEDGES>   d_graph_type;
  typedef dynamic_graph<UNDIRECTED, MULTIEDGES> u_graph_type;

  test_newman_watts_strogatz<d_graph_type>(n, k, p, " [Directed Graph]   ");
  test_newman_watts_strogatz<u_graph_type>(n, k, p, " [UnDirected Graph] ");

  test_watts_strogatz<d_graph_type>(n, k, p, " [Directed Graph]   ");
  test_watts_strogatz<u_graph_type>(n, k, p, " [UnDirected Graph] ");

  return EXIT_SUCCESS;
}

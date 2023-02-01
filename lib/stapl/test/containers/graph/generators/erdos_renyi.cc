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
#include <stapl/containers/graph/generators/erdos_renyi.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../../../test_report.hpp"

using namespace stapl;
using namespace std;


template<typename GraphView>
void test_output(GraphView& v, size_t n, double p, std::string s)
{
  STAPL_TEST_REPORT(v.size() == n,
                    "Testing size of Erdos-Renyi  " + s);

  bool passed = v.num_edges() >= n*(n-1)*p/2;
  STAPL_TEST_REPORT(passed, "Testing degree of Erdos-Renyi " + s);

  rmi_fence();
}


template<typename Graph>
void test_erdos_renyi(size_t n, double p, std::string s)
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
  view_type v = generators::make_erdos_renyi<view_type>(n, p);
  v1 = generators::make_erdos_renyi<view_type>(v1, n, p);
  v2 = generators::make_erdos_renyi<view_type>(v2, n, p);

  test_output(v, n, p, s);
  test_output(v1, n, p, s);
  test_output(v2, n, p, s);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cout << "usage: " << argv[0] << " n p" << std::endl;
    exit(1);
  }

  // inputs
  const size_t n = atol(argv[1]);
  const double p = atof(argv[2]);

  // types for graph and generator
  typedef dynamic_graph<DIRECTED, MULTIEDGES>   d_graph_type;
  typedef dynamic_graph<UNDIRECTED, MULTIEDGES> u_graph_type;

  test_erdos_renyi<d_graph_type>(n, p, " [Directed Graph]   ");
  test_erdos_renyi<u_graph_type>(n, p, " [UnDirected Graph] ");

  return EXIT_SUCCESS;
}

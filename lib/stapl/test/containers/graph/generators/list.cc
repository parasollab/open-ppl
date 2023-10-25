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
#include <stapl/containers/graph/generators/list.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>

#include "../../../test_report.hpp"
#include "generator_utils.hpp"

using namespace stapl;
using namespace std;

template<typename GraphView>
void test_output(GraphView& v, size_t n, std::string s)
{
  STAPL_TEST_REPORT(v.size() == n, "List Generator: Testing size  " + s);

  boost::unordered_map<size_t, size_t> distribution =
    degree_distribution(v);

  bool passed;

  if (!v.is_directed())
    passed = distribution[1] == 2 && distribution[2] == n - 2;
  else
    passed = distribution[0] == 1 && distribution[1] == n - 1;

  STAPL_TEST_REPORT(passed, "List Generator: Testing degree" + s);

  rmi_fence();
}

template<typename Graph>
void test_list_gen(size_t n, std::string s)
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
  view_type v = generators::make_list<view_type>(n, false);
  v1 = generators::make_list<view_type>(v1, n, false);
  v2 = generators::make_list<view_type>(v2, n, false);

  test_output(v, n, s);
  test_output(v1, n, s);
  test_output(v2, n, s);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " n" << std::endl;
    exit(1);
  }

  // inputs
  const size_t n = atol(argv[1]);

  stapl_assert(n > 2, "validation requires non-degenerate case");

  // types for graph and generator
  typedef dynamic_graph<DIRECTED, MULTIEDGES>   d_graph_type;
  typedef dynamic_graph<UNDIRECTED, MULTIEDGES> u_graph_type;

  test_list_gen<d_graph_type>(n, " [Directed Graph]   ");
  test_list_gen<u_graph_type>(n, " [UnDirected Graph] ");

  return EXIT_SUCCESS;
}

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
#include <stapl/containers/graph/generators/random_neighborhood.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>

#include "../../../test_report.hpp"
#include "generator_utils.hpp"

using namespace stapl;
using namespace std;

template<typename GraphView>
void test_output(GraphView& v, size_t n, size_t ef, std::string s)
{
  size_t ref_edge_cnt = 0;
  if (v.is_directed())
    ref_edge_cnt = 2*n*ef;
  else
    ref_edge_cnt = n*ef;

  bool passed = false;
  if (v.num_edges() >= 0.9*double(ref_edge_cnt) &&
      v.num_edges() <= 1.1*double(ref_edge_cnt))
    passed = true;

  STAPL_TEST_REPORT(v.size() == n,
                    std::string("RN Generator: Testing size  " + s));
  STAPL_TEST_REPORT(passed, std::string("RN Generator: Testing |E|   " + s));

  rmi_fence();
}

template<typename Graph>
void test_random_neighborhood_gen(size_t n, size_t ef, size_t k, std::string s)
{
  typedef Graph                                      graph_type;
  typedef graph_view<graph_type>                     view_type;

  // create graph and view
  graph_type g1;
  rmi_fence();
  view_type v1(g1);

  graph_type g2(n);
  rmi_fence();
  view_type v2(g2);

  // generate vertices and edges in graph
  view_type v = generators::make_random_neighborhood<view_type>(n, ef, k);
  v1 = generators::make_random_neighborhood<view_type>(v1, n, ef, k);
  v2 = generators::make_random_neighborhood<view_type>(v2, n, ef, k);

  test_output(v, n, ef, s);
  test_output(v1, n, ef, s);
  test_output(v2, n, ef, s);

  rmi_fence();
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 4) {
    std::cout << "usage: " << argv[0] << " n ef k" << std::endl;
    exit(1);
  }

  // inputs
  const size_t n  = atol(argv[1]);
  const size_t ef = atol(argv[2]);
  const size_t k  = atol(argv[3]);

  stapl_assert(n > 1 && ef > 1, "validation requires non-degenerate case");

  // types for graph and generator
  typedef dynamic_graph<DIRECTED, MULTIEDGES>   d_graph_type;
  typedef dynamic_graph<UNDIRECTED, MULTIEDGES> u_graph_type;

  test_random_neighborhood_gen<d_graph_type>(n, ef, k, " [Directed Graph]   ");
  test_random_neighborhood_gen<u_graph_type>(n, ef, k, " [UnDirected Graph] ");

  return EXIT_SUCCESS;
}

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
#include <stapl/containers/graph/generators/binary_tree_network.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../../../test_report.hpp"

using namespace stapl;
using namespace std;

struct verify_degree
{
  typedef bool result_type;

  template<typename Vertex>
  bool operator()(Vertex v)
  { return v.size() < 3; }
};


struct compare_target
{
  template<typename T, typename U>
  bool operator()(T const& x, U const& y)
  { return x.target() == y; }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  const size_t num_levels = 8;
  const size_t expected_size = 2*(pow(2, num_levels)-1);

  // types for graph and generator
  typedef dynamic_graph<DIRECTED, MULTIEDGES> graph_type;
  typedef graph_view<graph_type> view_type;

  // create graph and view
  graph_type g;
  view_type v(g);


  // generate vertices and edges in graph
  v = generators::make_binary_tree_network(v, num_levels);


  // verify size
  STAPL_TEST_REPORT(g.size() == expected_size, "Testing generated size");

  // verify binary tree-ness
  bool passed = map_reduce(verify_degree(), stapl::logical_and<bool>(), v);
  STAPL_TEST_REPORT(passed, "Testing out-degree")

  // verify edges of top tree vertex
  graph_type::vertex_reference::adj_edges_type edges = v[3].edges();
  size_t edges_top[2] = {7, 8};
  passed = equal(edges.begin(), edges.end(), edges_top, compare_target());
  STAPL_TEST_REPORT(passed, "Testing edges of top tree vertex");

  // verify edges of bottom tree vertex
  edges = v[256].edges();
  size_t edges_bottom[1] = {255};
  passed = equal(edges.begin(), edges.end(), edges_bottom, compare_target());
  STAPL_TEST_REPORT(passed, "Testing edges of bottom tree vertex");

  // verify edges of middle vertex
  edges = v[254].edges();
  size_t edges_middle[1] = {509};
  passed = equal(edges.begin(), edges.end(), edges_middle, compare_target());
  STAPL_TEST_REPORT(passed, "Testing edges of bottom tree vertex");

  // verify sink
  passed = v[255].size() == 0;
  STAPL_TEST_REPORT(passed, "Testing sink vertex");

  return EXIT_SUCCESS;
}

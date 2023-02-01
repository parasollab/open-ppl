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
#include <stapl/containers/graph/generators/binary_tree.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../../../test_report.hpp"


using namespace stapl;
using namespace std;

struct verify_degree
{
  typedef bool result_type;

  template<typename Vertex>
  bool operator()(Vertex v)
  { return v.size() == 0 || v.size() == 2; }
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
  const size_t expected_size = pow(2, num_levels)-1;

  // types for graph and generator
  typedef dynamic_graph<DIRECTED, MULTIEDGES> graph_type;
  typedef graph_view<graph_type> view_type;


  // create graph and view
  graph_type g;
  view_type v(g);

  // generate vertices and edges in graph
  v = generators::make_binary_tree(v, expected_size, false);

  // verify size
  STAPL_TEST_REPORT(g.size() == expected_size, "Testing generated size")

  // verify binary tree-ness
  bool passed = map_reduce(verify_degree(), stapl::logical_and<bool>(), v);

  STAPL_TEST_REPORT(passed, "Testing out-degree")

  // verify root
  graph_type::vertex_reference::adj_edges_type edges = v[0].edges();
  size_t edges_root[2] = {1, 2};

  passed = equal(edges.begin(), edges.end(), edges_root, compare_target());

  STAPL_TEST_REPORT(passed, "Testing edges root")

  // verify random vertex
  const size_t random = rand() % static_cast<size_t>(pow(2, num_levels-1)-1);
  edges = v[random].edges();
  size_t edges_rand[2] = {2*random + 1, 2*random + 2};

  passed = equal(edges.begin(), edges.end(), edges_rand, compare_target());

  STAPL_TEST_REPORT(passed, "Testing edges random")


  // verify leaf
  passed = v[150].size() == 0;
  STAPL_TEST_REPORT(passed, "Testing leaf vertex")

  return EXIT_SUCCESS;
}


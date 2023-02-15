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
#include <stapl/containers/graph/generators/random_dag.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>

#include "../../../test_report.hpp"
#include "generator_utils.hpp"

using namespace stapl;

struct all_edges_one_direction
{
  using result_type = bool;

  template<typename V>
  result_type operator()(V v) const
  {
    using edge_type = decltype(*v.begin());
    return std::all_of(v.begin(), v.end(), [](edge_type const& e) {
      return e.source() < e.target();
    });
  }
};


template<typename GraphView>
void test_output(GraphView& v, size_t n, std::size_t max_degree, std::string s)
{
  STAPL_TEST_REPORT(v.size() == n, "DAG Generator: Testing size " + s);
  STAPL_TEST_REPORT(v.num_edges() <= n*max_degree,
      "DAG Generator: Testing number of edges  " + s);

  bool passed = map_reduce(all_edges_one_direction{}, logical_and<bool>{}, v);

  STAPL_TEST_REPORT(passed, "DAG Generator: Testing acyclicity " + s);

  rmi_fence();
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " n max_degree" << std::endl;
    exit(1);
  }

  const size_t n = atol(argv[1]);
  const size_t max_degree = atol(argv[2]);

  stapl_assert(n > 2, "validation requires non-degenerate case");

  using graph_type = dynamic_graph<DIRECTED, MULTIEDGES>;
  using view_type = graph_view<graph_type>;

  // Create graph and view
  graph_type g1{};
  view_type v1{g1};

  // Generate vertices and edges in graph
  auto v = generators::make_random_dag<view_type>(n, max_degree);
  v1 = generators::make_random_dag<view_type>(v1, n, max_degree);

  test_output(v, n, max_degree, "auto");
  test_output(v1, n, max_degree, "generated");

  return EXIT_SUCCESS;
}

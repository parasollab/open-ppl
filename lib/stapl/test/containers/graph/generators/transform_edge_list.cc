/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/array.hpp>
#include <stapl/containers/graph/generators/edge_list.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../../../test_report.hpp"

using namespace stapl;

array_view<array<std::pair<std::size_t, std::size_t>>>
make_edge_list_view()
{
  auto* edges = new stapl::array<std::pair<std::size_t, std::size_t>>(7);

  stapl::do_once([edges]() {
    edges->set_element(0, std::make_pair(0, 1));
    edges->set_element(1, std::make_pair(0, 2));
    edges->set_element(2, std::make_pair(3, 2));
    edges->set_element(3, std::make_pair(1, 4));
    edges->set_element(4, std::make_pair(4, 1));
    edges->set_element(5, std::make_pair(0, 1));
    edges->set_element(6, std::make_pair(0, 3));
  });

  return {edges};
}

void test_forward_only()
{
  using graph_type = graph<DIRECTED, MULTIEDGES>;
  using view_type = graph_view<graph_type>;

  graph_type graph(5);
  view_type v(graph);

  auto edge_view = make_edge_list_view();
  transform_edge_list(edge_view, v);

  std::map<std::size_t, std::size_t> expected_sizes{
    {0,4},{1,1},{2,0},{3,1},{4,1}
  };

  for (auto&& expected : expected_sizes)
  {
    STAPL_TEST_REPORT(v[expected.first].size() == expected.second,
      "Testing vertex out-degrees (forward)");
  }

  bool passed = std::count_if(v[0].begin(), v[0].end(),
    [](decltype(*v[0].begin()) const& edge) {
      return edge.target() == 1;
    }) == 2;

  STAPL_TEST_REPORT(passed, "Testing vertex 0 has the repeated edge");
}

void test_both_directions()
{
  using graph_type = graph<DIRECTED, MULTIEDGES>;
  using view_type = graph_view<graph_type>;

  graph_type graph(5);
  view_type v(graph);

  auto edge_view = make_edge_list_view();
  transform_edge_list(edge_view, v, true);

  std::map<std::size_t, std::size_t> expected_sizes{
    {0,4},{1,4},{2,2},{3,2},{4,2}
  };

  for (auto&& expected : expected_sizes)
  {
    STAPL_TEST_REPORT(v[expected.first].size() == expected.second,
      "Testing vertex out-degrees (both)");
  }
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  test_forward_only();
  test_both_directions();

  return 0;
}

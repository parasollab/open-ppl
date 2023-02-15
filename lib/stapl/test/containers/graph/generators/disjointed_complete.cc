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
#include <stapl/containers/graph/generators/disjointed_complete.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "generator_utils.hpp"
#include "../../../test_report.hpp"

using namespace stapl;
using namespace std;

template <typename GraphView>
void test_output(GraphView& v,
                 size_t num_components,
                 size_t component_size,
                 std::string s)
{
  using edge_type = decltype(*v[0].begin());

  size_t degree = component_size-1;
  bool passed = map_reduce(degree_equal_to(degree), stapl::logical_and<bool>(),
                           v);
  STAPL_TEST_REPORT(passed, "Complete Generator: Testing degree" + s);

  passed = v.size() == num_components*component_size;
  STAPL_TEST_REPORT(passed, "Complete Generator: Testing size  " + s);

  // The first vertex should have edges from 1 to component_size-1
  passed = stapl::do_once([&](){
    std::vector<std::size_t> v0_edges(v[0].size());
    std::transform(v[0].begin(),
                   v[0].end(),
                   v0_edges.begin(),
                   [](edge_type const& e) { return e.target(); });

    std::sort(v0_edges.begin(), v0_edges.end());

    std::vector<std::size_t> expected_edges(component_size-1);
    std::iota(expected_edges.begin(), expected_edges.end(), 1);

    return std::equal(
      expected_edges.begin(), expected_edges.end(), v0_edges.begin()
    );
  });

  STAPL_TEST_REPORT(passed, "Complete Generator: Testing v0's edges  " + s);

  rmi_fence();
}

template <typename Graph>
void test_complete_gen(size_t component_size,
                       size_t num_components,
                       std::string s)
{
  typedef Graph                        graph_type;
  typedef graph_view<graph_type>       view_type;

  // create graph and view
  graph_type g1;
  rmi_fence();
  view_type v1(g1);

  graph_type g2(component_size*num_components);
  rmi_fence();
  view_type v2(g2);

  // generate vertices and edges in graph
  view_type v = generators::make_disjointed_complete<view_type>(num_components,
                                                                component_size);
  v1 = generators::make_complete<view_type>(v1, num_components, component_size);
  v2 = generators::make_complete<view_type>(v2, num_components, component_size);

  test_output(v, num_components, component_size, s);
  test_output(v1, num_components, component_size, s);
  test_output(v2, num_components, component_size, s);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " component_size num_components"
              << std::endl;
    exit(1);
  }

  // inputs
  const size_t component_size = atol(argv[1]);
  const size_t num_components = atol(argv[2]);

  // types for graph and generator
  using d_graph_type = dynamic_graph<DIRECTED, MULTIEDGES>;
  using u_graph_type = dynamic_graph<UNDIRECTED, MULTIEDGES>;

  test_complete_gen<d_graph_type>(
    component_size, num_components, " [Directed Graph]   ");
  test_complete_gen<u_graph_type>(
    component_size, num_components, " [UnDirected Graph] ");

  return EXIT_SUCCESS;
}

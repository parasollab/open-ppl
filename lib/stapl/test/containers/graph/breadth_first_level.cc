/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/breadth_first_level.hpp>
#include <stapl/containers/graph/generators/torus.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "test_util.h"

using namespace std;


struct bfl_validate_wf
{
  typedef bool result_type;
  template<typename V, typename G>
  bool operator()(V v, G& g)
  {
    auto source_level = v.property();
    // every edge in the input graph has vertices with levels that differ by
    // at most one or that both are not in the BFS tree:
    for (auto const& e : v) {
      auto target_level = g[e.target()].property();
      if (target_level > source_level+1) {
        return false;
      }
      // the BFS tree spans an entire connected component's vertices:
      if (source_level != 0 && target_level == 0) {
        return false;
      }
    }

    return true;
  }
};


struct extract_level_wf
{
  typedef void result_type;

  template<typename V, typename E>
  void operator() (V v, E e) const
  {
    e = v.property();
  }
};


template <class GraphView>
void test_core_graph(GraphView vgraph, size_t k)
{
  typedef GraphView graph_view_t;
  typedef typename GraphView::vertex_descriptor vd_type;

  typedef stapl::static_array<vd_type> array_t;
  typedef stapl::array_view<array_t> array_view_t;
  one_print("Testing Breadth First Search...\t\t");

  // level-sync BFS:
  auto exec_policy = stapl::sgl::make_execution_policy("lsync", vgraph);
  size_t iter1 = stapl::breadth_first_level(exec_policy, vgraph, 0);

  array_t result1_array(vgraph.size());
  array_view_t result1(result1_array);
  stapl::map_func(extract_level_wf(), vgraph, result1);

  // kla BFS:
  auto kla_exec_policy = stapl::sgl::make_execution_policy("kla", vgraph, k);
  size_t iter2 = stapl::breadth_first_level(kla_exec_policy, vgraph, 0);

  array_t result2_array(vgraph.size());
  array_view_t result2(result2_array);
  stapl::map_func(extract_level_wf(), vgraph, result2);

  bool passed = stapl::equal(result1, result2);
  if (k > 1)
    passed &= (iter2 < iter1);

  // H BFS:
  auto h_exec_policy = stapl::sgl::make_execution_policy("hier", vgraph);
  stapl::breadth_first_level(h_exec_policy, vgraph, 0);

  array_t result3_array(vgraph.size());
  array_view_t result3(result3_array);
  stapl::map_func(extract_level_wf(), vgraph, result3);
  passed &= stapl::equal(result1, result3);

  // Verify the BFS tree
  passed &= stapl::map_reduce(
    bfl_validate_wf(), stapl::logical_and<bool>(), vgraph,
    stapl::make_repeat_view(vgraph)
  );
  one_print(passed);
  stapl::rmi_fence();
}


stapl::exit_code stapl_main(int argc,char** argv)
{
  size_t nx, ny, k = 2;
  if (argc > 2) {
    nx = atol(argv[1]);
    ny = atol(argv[2]);
  } else {
    cout<<"usage: exe x-dim y-dim\n";
    return EXIT_FAILURE;
  }

  for (int i = 1; i < argc; i++) {
    if (!strcmp("--k", argv[i]))
      k = atoi(argv[i+1]);
  }

  typedef stapl::multidigraph<uint32_t> PGR_static;
  typedef stapl::graph_view<PGR_static> graph_view_t;
  graph_view_t vw = stapl::generators::make_torus<graph_view_t>(nx, ny);
  stapl::rmi_fence();

  test_core_graph(vw, k);

  return EXIT_SUCCESS;
}

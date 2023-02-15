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
#include <stapl/containers/graph/algorithms/sssp.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/torus.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "test_util.h"

using namespace std;

size_t TUNING = 2;

struct extract_level_wf
{
  typedef void result_type;

  template <typename V, typename E>
  void operator() (V v, E e)
  { e = v.property().distance(); }
};

struct init_edges_wf
{
  typedef void result_type;

  template <typename V>
  void operator() (V v)
  {
    for (typename V::adj_edge_iterator it = v.begin();
           it != v.end(); ++it)
      (*it).property() = rand() % 15;
  }
};


template <class GraphView>
void test_core_graph(GraphView vgraph)
{
  typedef GraphView graph_view_t;
  typedef typename GraphView::vertex_descriptor vd_type;

  typedef stapl::static_array<double> array_t;
  typedef stapl::array_view<array_t> array_view_t;

  one_print("Testing SSSP...\t\t\t\t");

  // Source vertex for traversal.
  vd_type source = 0;

  // level-sync SSSP:
  size_t iter1 = stapl::sssp(vgraph, source);

  array_t result1_array(vgraph.size());
  array_view_t result1(result1_array);
  stapl::map_func(extract_level_wf(), vgraph, result1);

  // kla SSSP:
  size_t iter2 = stapl::sssp(vgraph, source, TUNING);

  array_t result2_array(vgraph.size());
  array_view_t result2(result2_array);
  stapl::map_func(extract_level_wf(), vgraph, result2);

  bool passed = stapl::equal(result1, result2);
  if (TUNING > 1)
    passed &= (iter2 < iter1);

  one_print(passed);
  stapl::rmi_fence();
}


stapl::exit_code stapl_main(int argc,char** argv)
{
  size_t nx, ny;
  if (argc > 2) {
    nx = atol(argv[1]);
    ny = atol(argv[2]);
  } else {
    cout<<"usage: exe x-dim y-dim [--tuning k]\n";
    return EXIT_FAILURE;
  }

  srand(0);

  for (int i = 3; i < argc; i++) {
    if (!strcmp("--tuning", argv[i]))
      TUNING = atoi(argv[i+1]);
  }

  typedef stapl::multidigraph<stapl::properties::sssp_property,
                              double> PGR_static;
  typedef stapl::graph_view<PGR_static> graph_view_t;
  graph_view_t vw = stapl::generators::make_torus<graph_view_t>(nx, ny);

  map_func(init_edges_wf(), vw);

  test_core_graph(vw);

  return EXIT_SUCCESS;
}

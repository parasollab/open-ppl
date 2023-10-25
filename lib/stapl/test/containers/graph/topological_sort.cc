/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <stapl/runtime.hpp>
#include <stapl/containers/graph/digraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/topological_sort.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/binary_tree.hpp>

#include <cmath>
#include <iostream>
#include "test_util.h"

using namespace stapl;
using namespace std;

struct get_preds_wf
{
  typedef int result_type;

  template<typename Vertex>
  int operator()(Vertex v)
  {
    return v.property().preds();
  }
};


struct test_top_sort_binary_tree_mf
{
  typedef bool result_type;

  template<typename Vertex>
  bool operator()(Vertex v) const
  {
    return (v.property().rank() == floor(log2(v.descriptor()+1)));
  }
};


template<typename Graph>
void test_top_sort(int num_verts_per_loc, int k)
{
  one_print("Testing topological_sort...\t\t");

  typedef graph_view<Graph> graph_view_type;
  int n = num_verts_per_loc*get_num_locations();
  graph_view_type gvw = generators::make_binary_tree<graph_view_type>(n, false);
  topological_sort(gvw, k);

  bool passed_bin_tree_test =
         map_reduce(test_top_sort_binary_tree_mf(), stapl::logical_and<bool>(),
                    gvw);

  one_print(passed_bin_tree_test);
}


template<typename Graph>
void time_top_sort(int num_verts_per_loc, int k)
{
  stapl::counter<stapl::default_timer> tm;

  typedef graph_view<Graph> graph_view_type;
  int n = num_verts_per_loc*get_num_locations();

  rmi_fence();
  tm.reset();
  tm.start();
  graph_view_type gvw = generators::make_binary_tree<graph_view_type>(n, false);
  double pop_graph_time = tm.stop();

  tm.reset();
  tm.start();
  topological_sort(gvw, k);
  double algo_time = tm.stop();

  int num_predss = map_reduce(get_preds_wf(), stapl::max<int>(), gvw) + 1;
  if (get_location_id() == 0) {
    cout << "The number of vertices in the tree is: " << num_verts_per_loc
         << endl;
    cout << "The number of predss in the tree is: " << num_predss << endl;
    cout << "It took (" << pop_graph_time << ") to populate the graph." << endl;
    cout << "It took (" << algo_time
         << ") to finish topological sort on the graph." << endl;
  }
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef stapl::digraph<stapl::properties::topological_sort_property> Graph;
  int k = 0;
  int num_verts_per_loc = 1023;
  int option = 1;//1 for correctness test and 2 for timing
  if (argc > 1)
    option = atoi(argv[1]);
  if (argc > 2)
    num_verts_per_loc = atoi(argv[2]);
  if (argc > 3)
    k = atoi(argv[3]);

  if (option == 1)
    test_top_sort<Graph>(num_verts_per_loc, k);
  else
    time_top_sort<Graph>(num_verts_per_loc, k);

  return EXIT_SUCCESS;
}

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
#include <stapl/containers/graph/algorithms/mssp.hpp>
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
  { e = v.property().distances(); }
};


struct compare_wf
{
  typedef bool result_type;
  template <typename E1, typename E2>
  bool operator() (E1 e1, E2 e2)
  {
    boost::unordered_map<size_t, double> a = e1;
    boost::unordered_map<size_t, double> b = e2;
    if (a.size() != b.size())
      return false;
    boost::unordered_map<size_t, double>::iterator it, it2;
    for (it = a.begin(); it != a.end(); ++it) {
      it2 = b.find(it->first);
      if (it2 == b.end() || it2->second != it->second)
        return false;
    }
    return true;
  }
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
void test_core_graph(GraphView vgraph, size_t num_sources)
{
  typedef GraphView graph_view_t;
  typedef typename GraphView::vertex_descriptor vd_type;
  typedef boost::unordered_map<vd_type, double> distances_cont_t;
  typedef stapl::static_array<distances_cont_t> array_t;
  typedef stapl::array_view<array_t> array_view_t;

  one_print("Testing MSSP...\t\t\t\t");

  // sources:
  std::vector<size_t> sources;
  for (size_t i=0; i<num_sources; ++i) {
    vd_type s = rand() % vgraph.size();
    while (std::find(sources.begin(), sources.end(), s) != sources.end())
       s = rand() % vgraph.size();
    sources.push_back(s);
  }

  // level-sync MSP:
  stapl::mssp(vgraph, sources, 0);

  array_t result1_array(vgraph.size());
  array_view_t result1(result1_array);
  stapl::map_func(extract_level_wf(), vgraph, result1);

  // kla MSSP:
  stapl::mssp(vgraph, sources, TUNING);

  array_t result2_array(vgraph.size());
  array_view_t result2(result2_array);
  stapl::map_func(extract_level_wf(), vgraph, result2);

  bool passed = stapl::map_reduce(compare_wf(), stapl::logical_and<bool>(),
                                  result1, result2);

  one_print(passed);
  stapl::rmi_fence();
}


stapl::exit_code stapl_main(int argc,char** argv)
{
  size_t nx, ny;
  size_t num_sources = 3;
  if (argc > 2) {
    nx = atol(argv[1]);
    ny = atol(argv[2]);
  } else {
    cout<<"usage: exe x-dim y-dim [--tuning k] [--num_sources s]\n";
    return EXIT_FAILURE;
  }

  srand(0);

  for (int i = 3; i < argc; i++) {
    if (!strcmp("--tuning", argv[i]))
      TUNING = atoi(argv[i+1]);
    if (!strcmp("--num_sources", argv[i]))
      num_sources = atoi(argv[i+1]);
  }

  typedef stapl::multidigraph<stapl::properties::mssp_property,
                              double> PGR_static;
  typedef stapl::graph_view<PGR_static> graph_view_t;
  graph_view_t vw = stapl::generators::make_torus<graph_view_t>(nx, ny);

  map_func(init_edges_wf(), vw);

  test_core_graph(vw, num_sources);

  return EXIT_SUCCESS;
}

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
#include <stapl/containers/graph/generators/cycles.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>

#include "../../../test_report.hpp"
#include "generator_utils.hpp"

using namespace stapl;

template<typename GraphView>
void test_output(GraphView& v, size_t cyc_count, size_t cyc_size,
                 bool connected, bool bidir, std::string s)
{
  STAPL_TEST_REPORT(v.size() == cyc_count*cyc_size,
                    std::string("Cycle Generator: Testing size  " + s));

  boost::unordered_map<size_t, size_t> distribution =
    degree_distribution(v);

  bool passed = false;
  if (connected) {
    if (bidir) {
      passed  = (distribution.size() == 2);
      passed &= (distribution[2] == cyc_count*cyc_size-2*cyc_count+2);
      passed &= (distribution[3] == 2*cyc_count-2);
    } else {
      passed  = (distribution.size() == 2);
      passed &= (distribution[1] == cyc_count*cyc_size-cyc_count+1);
      passed &= (distribution[2] == cyc_count-1);
    }
  } else {
    if (bidir) {
      passed  = (distribution.size() == 1);
      passed &= (distribution[2] == cyc_count*cyc_size);
    } else {
      passed  = (distribution.size() == 1);
      passed &= (distribution[1] == cyc_count*cyc_size);
    }
  }

  STAPL_TEST_REPORT(passed, std::string("Cycle Generator: Testing degree" + s));

  rmi_fence();
}

template<typename Graph>
void test_cycle_gen(size_t cyc_count, size_t cyc_size, bool connected,
                    bool bidir, std::string s)
{
  using namespace generators;

  typedef Graph                        graph_type;
  typedef graph_view<graph_type>       view_type;

  // create graph and view
  graph_type g1;
  rmi_fence();
  view_type v1(g1);

  graph_type g2(cyc_count*cyc_size);
  rmi_fence();
  view_type v2(g2);

  // generate vertices and edges in graph
  view_type v;

  if (connected) {
    v  = make_cycle_chain<view_type>(cyc_count, cyc_size, bidir);
    v1 = make_cycle_chain<view_type>(v1, cyc_count, cyc_size, bidir);
    v2 = make_cycle_chain<view_type>(v2, cyc_count, cyc_size, bidir);
  } else {
    v  = make_unconnected_cycles<view_type>(cyc_count, cyc_size, bidir);
    v1 = make_unconnected_cycles<view_type>(v1, cyc_count, cyc_size, bidir);
    v2 = make_unconnected_cycles<view_type>(v2, cyc_count, cyc_size, bidir);
  }

  test_output(v, cyc_count, cyc_size, connected, bidir, s);
  test_output(v1, cyc_count, cyc_size, connected, bidir, s);
  test_output(v2, cyc_count, cyc_size, connected, bidir, s);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cout << "usage: " << argv[0] << " cyc_count cyc_size" << std::endl;
    exit(1);
  }

  // inputs
  const size_t cyc_count = atol(argv[1]);
  const size_t cyc_size = atol(argv[2]);

  if (!(cyc_count > 1 && cyc_size > 1)) {
    std::cout << "validation requires non-degenerate case" << std::endl;
    rmi_fence();
    exit(0);
  }

  // types for graph and generator
  typedef dynamic_graph<DIRECTED, MULTIEDGES>   d_graph_type;
  typedef dynamic_graph<UNDIRECTED, MULTIEDGES> u_graph_type;

  test_cycle_gen<d_graph_type>(cyc_count, cyc_size, false, true,
                               " unconnected [Directed Graph]   ");
  test_cycle_gen<u_graph_type>(cyc_count, cyc_size, false, true,
                               " unconnected [UnDirected Graph] ");

  test_cycle_gen<d_graph_type>(cyc_count, cyc_size, true, true,
                               "   connected [Directed Graph]   ");
  test_cycle_gen<u_graph_type>(cyc_count, cyc_size, true, true,
                               "   connected [UnDirected Graph] ");

  test_cycle_gen<d_graph_type>(cyc_count, cyc_size, false, false,
                               " unconnected-uni [Directed Graph]   ");
  test_cycle_gen<d_graph_type>(cyc_count, cyc_size, true, false,
                               "   connected-uni [Directed Graph]   ");

  return EXIT_SUCCESS;
}

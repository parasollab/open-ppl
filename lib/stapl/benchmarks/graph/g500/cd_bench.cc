/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Benchmark suite for Community Detection (CD) algorithm on the Graph500
/// benchmark input (graph500.org).
///
/// The benchmark calls the graph500 generator from the benchmark provided code
/// (compilation and linking against it is required) to generate an array of
/// edges represented as an array of <tt>2*E ints</tt>, where even indices are
/// sources and odd ones are targets for the edges. The array is local to each
/// process, so communication is required to build the graph.
///
/// Finally, Community Detection is called on the graph and reports
/// the throughput in MTEPS (Mega Traversed Edges per Second).
///
/// The algorithm can alternatively be run using a hierarchical or hubs CD
/// (@ref community_detection(GView, HView),
/// @ref community_detection(GView, HView, HubsView)) for better scalability.
//////////////////////////////////////////////////////////////////////

#include "g500.h"
#include <benchmarks/lonestar/utility/test_util.h>

#include <stapl/containers/graph/algorithms/community_detection.hpp>

#include <iostream>
#include <cstring>


struct cd_helper
{
  size_t m_max_iter;

  cd_helper(size_t max_iter)
    : m_max_iter(max_iter)
  { }

  typedef stapl::multidigraph<stapl::properties::cd_property>  graph_t;

  template<typename Graph>
  size_t operator()(Graph& g)
  {
    stapl::do_once([] { std::cout << "starting CD " << std::endl; });

    auto policy = stapl::sgl::execution_policy<Graph>{
      stapl::sgl::level_sync_policy{}
    };

    return stapl::community_detection(policy, g, m_max_iter);
  }

  std::string name(void)
  { return "cd"; }
};


struct cd_h_helper
{
  size_t m_max_iter;

  cd_h_helper(size_t max_iter)
    : m_max_iter(max_iter)
  { }

  typedef stapl::multidigraph<stapl::properties::cd_property>  graph_t;

  template<typename Graph, typename HGraph>
  size_t operator()(Graph& g, HGraph& h)
  {
    stapl::do_once([] { std::cout << "starting CD " << std::endl; });

    auto policy = stapl::sgl::execution_policy<Graph>{
      stapl::sgl::hierarchical_policy<Graph>{h}
    };

    return stapl::community_detection(policy, g, m_max_iter);
  }

  std::string name(void)
  { return "cd_hierarchical"; }
};


struct cd_hubs_helper
{
  size_t m_max_iter;

  cd_hubs_helper(size_t max_iter)
    : m_max_iter(max_iter)
  { }

  typedef stapl::multidigraph<stapl::properties::cd_property>  graph_t;

  template<typename Graph, typename HGraph, typename HubsGraph>
  size_t operator()(Graph& g, HGraph& h, HubsGraph& hubs)
  {
    stapl::do_once([] { std::cout << "starting CD " << std::endl; });

    auto policy = stapl::sgl::execution_policy<Graph>{
      stapl::sgl::hierarchical_hubs_policy<Graph>{ h, hubs, 0 }
    };

    return stapl::community_detection(policy, g, m_max_iter);
  }

  std::string name(void)
  { return "cd_hierarchical_hubs"; }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t ef = 16;
  size_t SCALE = 12;
  size_t NEXP = 1;
  size_t hub_sz = 1024;
  size_t max_iter = 10;
  std::string option = "";

  if (argc > 1) {
    SCALE = boost::lexical_cast<size_t>(argv[1]);
    ef = boost::lexical_cast<size_t>(argv[2]);
  } else {
    std::cout << "SCALE N and EdgeFactor EF required; Using 12 16 by default"
              << std::endl;
  }

  for (int i = 1; i < argc; i++) {
    if (!strcmp("--miniterations", argv[i]))
      NEXP = boost::lexical_cast<size_t>(argv[i+1]);
    if (!strcmp("--hierarchical", argv[i]))
      option = "hierarchical";
    if (!strcmp("--hubs", argv[i]))
      option = "hubs";
    if (!strcmp("--hub_size", argv[i]))
      hub_sz = boost::lexical_cast<size_t>(argv[i+1]);
    if (!strcmp("--max_iter", argv[i]))
      max_iter = boost::lexical_cast<size_t>(argv[i+1]);
 }

  const size_t nv = std::pow(2.0, double(SCALE));
  auto input_graph
    = graph500_benchmark_generator<cd_helper>(nv, ef, SCALE);
  size_t nedges = input_graph.num_edges_collective();

  // call the CD experiments.
  if (option == "hierarchical") {
    auto h = create_level_machine_helper(input_graph);
    run_algo(NEXP, nedges, cd_h_helper(max_iter), input_graph, h);
  } else if (option == "hubs") {
    auto hubs = create_level_hubs_helper(input_graph, hub_sz);
    auto h = create_level_machine_helper(input_graph);
    run_algo(NEXP, nedges, cd_hubs_helper(max_iter), input_graph, h, hubs);
  } else {
    run_algo(NEXP, nedges, cd_helper(max_iter), input_graph);
  }

  return EXIT_SUCCESS;
}

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
/// Benchmark suite for PageRank algorithm on the Graph500 benchmark
/// input (graph500.org).
///
/// The benchmark calls the graph500 generator from the benchmark provided code
/// (compilation and linking against it is required) generate an array of edges
/// represented as an array of <tt>2*E ints</tt>, where even indices are sources
/// and odd ones are targets for the edges. The array is local to each process,
/// so communication is required to build the graph.
///
/// Finally, PageRank is called on the graph and reports
/// the throughput in MTEPS (Mega Traversed Edges per Second).
///
/// The algorithm can alternatively be run using a hierarchical or hubs PageRank
/// for better scalability.
//////////////////////////////////////////////////////////////////////

#include "g500.h"
#include <benchmarks/lonestar/utility/test_util.h>

#include <stapl/containers/graph/algorithms/page_rank.hpp>

#include <iostream>
#include <cstring>


struct pr_helper
{
  using graph_t =
    stapl::multidigraph<stapl::properties::page_rank_property<float>>;

  size_t m_num_iter;

  pr_helper(size_t num_iter)
    : m_num_iter(num_iter)
  { }

  template<typename Graph>
  size_t operator()(Graph& g)
  {
    stapl::do_once([] { std::cout << "starting PR " << std::endl; });

    return stapl::page_rank(g, m_num_iter);
  }

  std::string name(void)
  { return "pr"; }
};


struct pr_h_helper
{
  using graph_t =
    stapl::multidigraph<stapl::properties::page_rank_property<float>>;

  size_t m_num_iter;

  pr_h_helper(size_t num_iter)
    : m_num_iter(num_iter)
  { }

  template<typename Graph, typename HGraph>
  size_t operator()(Graph& g, HGraph& h)
  {
    stapl::do_once([] { std::cout << "starting PR " << std::endl; });

    auto policy = stapl::sgl::execution_policy<Graph>{
      stapl::sgl::hierarchical_policy<Graph>{h}};
    return stapl::page_rank(policy, g, m_num_iter);
  }

  std::string name(void)
  { return "pr_hierarchical"; }
};


struct pr_hubs_helper
{
  using graph_t =
    stapl::multidigraph<stapl::properties::page_rank_property<float>>;

  size_t m_num_iter;

  pr_hubs_helper(size_t num_iter)
    : m_num_iter(num_iter)
  { }

  template<typename Graph, typename HGraph, typename HubsGraph>
  size_t operator()(Graph& g, HGraph& h, HubsGraph& hubs)
  {
    stapl::do_once([] { std::cout << "starting PR " << std::endl; });

    auto policy = stapl::sgl::execution_policy<Graph>{
      stapl::sgl::hierarchical_hubs_policy<Graph>{h, hubs, 0}};
    return stapl::page_rank(policy, g, m_num_iter);
  }

  std::string name(void)
  { return "pr_hierarchical_hubs"; }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t ef = 16;
  size_t SCALE = 12;
  size_t NEXP = 1;
  size_t hub_sz = 1024;
  size_t pr_iter = 20;
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
    if (!strcmp("--pr_iter", argv[i]))
      pr_iter = boost::lexical_cast<size_t>(argv[i+1]);
 }

  const size_t nv = std::pow(2.0, double(SCALE));
  auto input_graph
    = graph500_benchmark_generator<pr_helper>(nv, ef, SCALE);
  size_t nedges = input_graph.num_edges_collective();

  // call the PR experiments.
  if (option == "hierarchical") {
    auto h = create_level_machine_helper(input_graph);
    run_algo(NEXP, nedges, pr_h_helper(pr_iter), input_graph, h);
  } else if (option == "hubs") {
    auto hubs = create_level_hubs_helper(input_graph, hub_sz);
    auto h = create_level_machine_helper(input_graph);
    run_algo(NEXP, nedges, pr_hubs_helper(pr_iter), input_graph, h, hubs);
  } else {
    run_algo(NEXP, nedges, pr_helper(pr_iter), input_graph);
  }

  return EXIT_SUCCESS;
}

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
/// Benchmark suite for the k-core decomposition algorithm on the
/// Graph500 benchmark input (graph500.org).
///
/// The benchmark calls the graph500 generator from the benchmark provided code
/// (compilation and linking against it is required) to generate an array of
/// edges represented as an array of <tt>2*E ints</tt>, where even indices are
/// sources and odd ones are targets for the edges. The array is local to each
/// process, so communication is required to build the graph.
///
/// Finally, k-core decomposition is called on the graph and reports
/// the throughput in MTEPS (Mega Traversed Edges per Second).
///
/// The algorithm can alternatively be run using a hierarchical or hubs k-core
/// (@ref k_core_h()) for better scalability.
//////////////////////////////////////////////////////////////////////

#include "g500.h"
#include <benchmarks/lonestar/utility/test_util.h>

#include <stapl/containers/graph/algorithms/k_core.hpp>

#include <iostream>
#include <cstring>


struct count_cores_wf
{
  typedef int result_type;
  template<typename V>
  int operator()(V v) const
  { return v.property() > 0; }
};


struct kc_helper
{
  typedef stapl::multidigraph<int>  graph_t;

  size_t m_core_sz;
  size_t m_k;

  kc_helper(size_t core_sz, size_t k)
    : m_core_sz(core_sz), m_k(k)
  { }

  template<typename Graph>
  size_t operator()(Graph& g)
  {
    if (stapl::get_location_id() == 0)
      std::cout << "starting KC " << std::endl;

    auto policy = stapl::sgl::execution_policy<Graph>{
      stapl::sgl::kla_policy{m_k}};
    stapl::k_core(policy, g, m_core_sz);

    size_t y = stapl::map_reduce(count_cores_wf(), stapl::plus<int>(), g);

    return y;
  }

  std::string name(void)
  { return "kc"; }
};


struct kc_h_helper
{
  typedef stapl::multidigraph<int>  graph_t;

  size_t m_core_sz;

  kc_h_helper(size_t core_sz)
    : m_core_sz(core_sz)
  { }

  template<typename Graph, typename HGraph>
  size_t operator()(Graph& g, HGraph& h)
  {
    if (stapl::get_location_id() == 0)
      std::cout << "starting KC " << std::endl;

    auto policy = stapl::sgl::execution_policy<Graph>{
      stapl::sgl::hierarchical_policy<Graph>{h}};
    stapl::k_core(policy, g, m_core_sz);

    size_t y = stapl::map_reduce(count_cores_wf(), stapl::plus<int>(), g);

    return y;
  }

  std::string name(void)
  { return "kc_hierarchical"; }
};


struct kc_hubs_helper
{
  typedef stapl::multidigraph<int>  graph_t;

  size_t m_core_sz;

  kc_hubs_helper(size_t core_sz)
    : m_core_sz(core_sz)
  { }

  template<typename Graph, typename HGraph, typename HubsGraph>
  size_t operator()(Graph& g, HGraph& h, HubsGraph& hubs)
  {
    if (stapl::get_location_id() == 0)
      std::cout << "starting KC " << std::endl;

    auto policy = stapl::sgl::execution_policy<Graph>{
      stapl::sgl::hierarchical_hubs_policy<Graph>{h, hubs, 0}};
    stapl::k_core(policy, g, m_core_sz);

    size_t y = stapl::map_reduce(count_cores_wf(), stapl::plus<int>(), g);

    return y;
  }

  std::string name(void)
  { return "kc_hierarchical_hubs"; }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t ef = 16;
  size_t SCALE = 12;
  size_t NEXP = 1;
  size_t hub_sz = 1024;
  size_t core_sz = 1024;
  size_t k = 0;
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
    if (!strcmp("--core_size", argv[i]))
      core_sz = boost::lexical_cast<size_t>(argv[i+1]);
    if (!strcmp("--k", argv[i]))
      k = boost::lexical_cast<size_t>(argv[i+1]);
 }

  const size_t nv = std::pow(2.0, double(SCALE));
  auto input_graph
    = graph500_benchmark_generator<kc_helper>(nv, ef, SCALE, true);
  size_t nedges = input_graph.num_edges_collective();

  // call the KC experiments.
  if (option == "hierarchical") {
    auto h = create_level_machine_helper(input_graph);
    run_algo(NEXP, nedges, kc_h_helper(core_sz), input_graph, h);
  } else if (option == "hubs") {
    auto hubs = create_level_hubs_helper(input_graph, hub_sz);
    auto h = create_level_machine_helper(input_graph);
    run_algo(NEXP, nedges, kc_hubs_helper(core_sz), input_graph, h, hubs);
  } else {
    run_algo(NEXP, nedges, kc_helper(core_sz, k), input_graph);
  }

  return EXIT_SUCCESS;
}

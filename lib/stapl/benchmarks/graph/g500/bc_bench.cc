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
/// Benchmark suite for Betweenness Centrality (BC) algorithm on the Graph500
/// benchmark input (graph500.org).
///
/// The benchmark calls the graph500 generator from the benchmark provided code
/// (compilation and linking against it is required) to generate an array of
/// edges represented as an array of <tt>2*E ints</tt>, where even indices are
/// sources and odd ones are targets for the edges. The array is local to each
/// process, so communication is required to build the graph.
///
/// Finally, BC is called on the graph from randomly chosen sources and reports
/// the throughput in MTEPS (Mega Traversed Edges per Second).
///
/// The benchmark can alternatively be run using a hierarchical BC
/// for better scalability.
//////////////////////////////////////////////////////////////////////

#include "g500.h"
#include <benchmarks/lonestar/utility/test_util.h>

#include <stapl/containers/graph/algorithms/betweenness_centrality.hpp>

#include <iostream>
#include <cstring>


struct bc_helper
{
  typedef stapl::multidigraph<stapl::properties::bc_property>  graph_t;

  size_t m_sources;

  bc_helper(size_t const& sources)
    : m_sources(sources)
  { }

  template<typename Graph>
  size_t operator()(Graph& g)
  {
    auto sources = m_sources;
    stapl::do_once(
      [sources] {
        std::cout << "starting BC from " << sources << " sources\n";
      });

    stapl::betweenness_centrality(g, m_sources);
    return 1;
  }

  std::string name(void)
  { return "bc"; }
};


struct bc_h_helper
{
  typedef stapl::multidigraph<stapl::properties::bc_property>  graph_t;

  size_t m_sources;

  bc_h_helper(size_t const& sources)
    : m_sources(sources)
  { }

  template<typename Graph, typename HGraph>
  size_t operator()(Graph& g, HGraph& h)
  {
    auto sources = m_sources;
    stapl::do_once(
      [sources] {
        std::cout << "starting BC from " << sources << " sources\n";
      });

    auto policy = stapl::sgl::execution_policy<Graph>{
      stapl::sgl::hierarchical_policy<Graph>{h}};
    stapl::betweenness_centrality(policy, g, m_sources);
    return 1;
  }


  std::string name(void)
  { return "bc_hierarchical"; }
};


template<typename Graph>
void report(Graph& g)
{
  stapl::do_once(
      [g] {
    size_t x = 1;
    std::stringstream ss;
    ss << " { ";
    for (size_t i=0; i<4; ++i) {
      ss << g[x].property().BC() << ". ";
      x *= 2;
    }
    ss << " }";
    std::cout << ss.str() << std::endl;
      });
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t ef = 16;
  size_t SCALE = 12;
  size_t NEXP = 1;
  std::string option;
  size_t sources = 0;

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
    if (!strcmp("--sources", argv[i]))
      sources = boost::lexical_cast<size_t>(argv[i+1]);
  }

  const size_t nv = std::pow(2.0, double(SCALE));
  auto input_graph
    = graph500_benchmark_generator<bc_helper>(nv, ef, SCALE);
  const size_t nedges = input_graph.num_edges_collective();

  // call the BC experiments.
  if (option == "hierarchical") {
    auto h = create_level_machine_helper(input_graph);
    run_algo(NEXP, nedges, bc_h_helper(sources),
             input_graph, h);
  } else {
    run_algo(NEXP, nedges, bc_helper(sources), input_graph);
  }

  report(input_graph);

  return EXIT_SUCCESS;
}

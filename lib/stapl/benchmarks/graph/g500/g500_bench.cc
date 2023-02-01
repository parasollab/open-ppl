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
/// Benchmark suite for the Graph500 benchmark (graph500.org).
///
/// The benchmark calls the graph500 generator from the benchmark provided code
/// (compilation and linking against it is required) generate an array of edges
/// represented as an array of <tt>2*E ints</tt>, where even indices are sources
/// and odd ones are targets for the edges. The array is local to each process,
/// so communication is required to build the graph.
///
/// Finally, BFS is called on the graph from randomly chosen sources and reports
/// the throughput in MTEPS (Mega Traversed Edges per Second).
///
/// The benchmark can alternatively be run using a hierarchical or hubs BFS
/// for better scalability.
//////////////////////////////////////////////////////////////////////

#include "g500.h"
#include <benchmarks/lonestar/utility/test_util.h>

#include <stapl/containers/graph/algorithms/breadth_first_search.hpp>

#include <iostream>
#include <cstring>


struct bfs_validate_wf
{
  typedef bool result_type;
  template<typename V, typename G>
  bool operator()(V v, G& g)
  {
    auto source_level = v.property().level();
    auto parent = v.property().parent();
    // each tree edge connects vertices whose BFS levels differ by exactly one,
    // or both the parent and the current vertex are not part of the BFS tree
    // (level=0) or current vertex is the source:
    if (parent != v.descriptor())
      if (g[parent].property().level() != source_level - 1)
        if (!(g[parent].property().level() == 0 &&
              source_level == 0))
          return false;

    // every edge in the input graph has vertices with levels that differ by
    // at most one or that both are not in the BFS tree:
    for (auto const& e : v) {
      auto target_level = g[e.target()].property().level();
      if (target_level > source_level+1)
        return false;
      // the BFS tree spans an entire connected component's vertices:
      if (source_level != 0 && target_level == 0)
        return false;
    }

    return true;
  }
};


template<typename Graph>
void validate_bfs(Graph& g)
{
  bool correct
    = stapl::map_reduce(bfs_validate_wf(), stapl::logical_and<bool>(),
                        g, stapl::make_repeat_view(g));
  if (correct)
    stapl_print("[PASSED]\n");
  else
    stapl_print("[FAILED]\n");
}


template<typename G>
std::vector<size_t> select_sources(G const& g, size_t NEXP)
{
  std::vector<size_t> sources;
  sources.reserve(NEXP);

  // Select sources for BFS. Must have >0 outgoing edges to avoid messing the
  // experiments with 0.0 times. NEXP number of sources will be selected.
  boost::random::mt19937 gen;
  boost::random::uniform_int_distribution<size_t> dist(0, g.size()-1);
  while (sources.size() < NEXP) {
    size_t t = dist(gen);
    if ((*(g.find_vertex(t))).size() > 0)
      sources.push_back(t);
  }
  return sources;
}


struct bfs_helper
{
  typedef stapl::multidigraph<stapl::properties::bfs_property>  graph_t;

  std::vector<size_t> m_sources;
  size_t m_cnt;
  bool   m_validate;

  bfs_helper(std::vector<size_t> const& sources, bool validate = false)
    : m_sources(sources), m_cnt(0), m_validate(validate)
  { }

  template<typename Graph>
  size_t operator()(Graph& g)
  {
    size_t source = m_sources[m_cnt++];
    stapl::do_once(
      [source] { std::cout << "starting BFS from vertex " << source << '\n'; });

    auto policy = stapl::sgl::execution_policy<Graph>{
      stapl::sgl::level_sync_policy{}};
    size_t levels = stapl::breadth_first_search(policy, g, source);

    if (m_validate)
      validate_bfs(g);

    return levels;
  }

  std::string name(void)
  { return "bfs"; }
};


struct bfs_h_helper
{
  typedef stapl::multidigraph<stapl::properties::bfs_property>  graph_t;

  std::vector<size_t> m_sources;
  size_t m_cnt;
  bool   m_validate;

  bfs_h_helper(std::vector<size_t> const& sources, bool validate = false)
    : m_sources(sources), m_cnt(0), m_validate(validate)
  { }

  template<typename Graph, typename HGraph>
  size_t operator()(Graph& g, HGraph& h)
  {
    size_t source = m_sources[m_cnt++];
    stapl::do_once(
      [source] { std::cout << "starting BFS from vertex " << source << '\n'; });

    auto policy = stapl::sgl::execution_policy<Graph>{
      stapl::sgl::hierarchical_policy<Graph>{h}};
    size_t levels = stapl::breadth_first_search(policy, g, source);

    if (m_validate)
      validate_bfs(g);

    return levels;
  }


  std::string name(void)
  { return "bfs_hierarchical"; }
};



struct bfs_h2_helper
{
  typedef stapl::multidigraph<stapl::properties::bfs_property>  graph_t;

  std::vector<size_t> m_sources;
  size_t m_cnt;
  bool   m_validate;

  bfs_h2_helper(std::vector<size_t> const& sources, bool validate = false)
    : m_sources(sources), m_cnt(0), m_validate(validate)
  { }

  template<typename Graph, typename H1Graph, typename H2Graph>
  size_t operator()(Graph& g, H1Graph& h1, H2Graph& h2)
  {
    size_t source = m_sources[m_cnt++];
    stapl::do_once(
      [source] { std::cout << "starting BFS from vertex " << source << '\n'; });

    auto policy = stapl::sgl::execution_policy<Graph>{
      stapl::sgl::hierarchical2_policy<Graph>{h1, h2}};
    size_t levels = stapl::breadth_first_search(policy, g, source);

    if (m_validate)
      validate_bfs(g);

    return levels;
  }


  std::string name(void)
  { return "bfs_hierarchical_2level"; }
};


struct bfs_hubs_helper
{
  typedef stapl::multidigraph<stapl::properties::bfs_property>  graph_t;

  std::vector<size_t> m_sources;
  size_t m_cnt;
  bool   m_validate;

  bfs_hubs_helper(std::vector<size_t> const& sources, bool validate = false)
    : m_sources(sources), m_cnt(0), m_validate(validate)
  { }

  template<typename Graph, typename HGraph, typename HubsGraph>
  size_t operator()(Graph& g, HGraph& h, HubsGraph& hubs)
  {
    size_t source = m_sources[m_cnt++];
    stapl::do_once(
      [source] { std::cout << "starting BFS from vertex " << source << '\n'; });

    auto policy = stapl::sgl::execution_policy<Graph>{
      stapl::sgl::hierarchical_hubs_policy<Graph>{h, hubs, 0}};
    size_t levels = stapl::breadth_first_search(policy, g, source);

    if (m_validate)
      validate_bfs(g);

    return levels;
  }

  std::string name(void)
  { return "bfs_hierarchical_hubs"; }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t ef = 16;
  size_t SCALE = 12;
  size_t NEXP = 1;
  size_t hub_sz = 1024;
  std::string option;
  bool validate_per_run = false;

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
    if (!strcmp("--hierarchical2", argv[i]))
      option = "hierarchical2";
    if (!strcmp("--hubs", argv[i]))
      option = "hubs";
    if (!strcmp("--hub_size", argv[i]))
      hub_sz = boost::lexical_cast<size_t>(argv[i+1]);
    if (!strcmp("--validate_per_run", argv[i])) {
      validate_per_run = true;
      stapl_print("***WARNING****: Validations will be timed!\n");
    }
  }

  const size_t nv = std::pow(2.0, double(SCALE));
  auto input_graph
    = graph500_benchmark_generator<bfs_helper>(nv, ef, SCALE);
  const auto sources = select_sources(input_graph, NEXP);
  const size_t nedges = input_graph.num_edges_collective();

  // call the BFS experiments.
  if (option == "hierarchical") {
    auto h = create_level_machine_helper(input_graph);
    run_algo(NEXP, nedges, bfs_h_helper(sources, validate_per_run),
             input_graph, h);
  } else if (option == "hierarchical2") {
    auto h = create_level_machine_helper(input_graph);
    auto h2 = create_level2_machine_helper(h);

    run_algo(NEXP, nedges, bfs_h2_helper(sources, validate_per_run),
             input_graph, h, h2);
  } else if (option == "hubs") {
    auto hubs = create_level_hubs_helper(input_graph, hub_sz);
    auto h = create_level_machine_helper(input_graph);
    run_algo(NEXP, nedges, bfs_hubs_helper(sources, validate_per_run),
             input_graph, h, hubs);
  } else {
    run_algo(NEXP, nedges, bfs_helper(sources, validate_per_run), input_graph);
  }

  if (!validate_per_run)
    validate_bfs(input_graph);

  return EXIT_SUCCESS;
}

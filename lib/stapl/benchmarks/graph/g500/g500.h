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
/// Benchmark utilities for the Graph500 benchmark (graph500.org).
///
/// The file provides functions to call the graph500 generator from the
/// benchmark provided code (compilation and linking against it is required)
/// and generates a pGraph. It also provides helpers to create machines and
/// hubs hierarchies and to run algorithms on the input.
//////////////////////////////////////////////////////////////////////

#ifndef BENCHMARKS_G500_HPP
#define BENCHMARKS_G500_HPP

#include <stapl/array.hpp>
#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/algorithms/aggregator.hpp>
#include <stapl/containers/graph/algorithms/create_level_machine.hpp>
#include <stapl/containers/graph/algorithms/create_level_hubs.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/edge_list.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/skeletons/functional/scan.hpp>
#include <stapl/utility/do_once.hpp>

#include <benchmarks/lonestar/utility/test_util.h>

#include <boost/lexical_cast.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>

#include <iostream>
#include <cstring>

#include "generator/make_graph.h"

struct count_cut_edges_wf
{
  typedef size_t result_type;
  template<typename V, typename G>
  result_type operator()(V v, G const& g)
  {
    size_t remote_edge_count = 0;
    for (auto const& e : v)
      if (!g.container().is_local(e.target()))
        ++remote_edge_count;
    return remote_edge_count;
  }
};

template<typename G>
size_t count_cut_edges(G& g)
{
  return stapl::map_reduce(count_cut_edges_wf(), stapl::plus<size_t>(),
                           g, make_repeat_view(g));
}


//////////////////////////////////////////////////////////////////////
/// @brief Return where this location should start writing into the global
///        edge list based on the number of edges on the previous locations
///
/// @param num_g500_edges The number of edges generated on this location
//////////////////////////////////////////////////////////////////////
std::size_t local_edge_list_offset(std::size_t num_g500_edges)
{
  stapl::array<std::size_t> a(stapl::get_num_locations());
  stapl::array_view<stapl::array<std::size_t>> v(a);

  v[stapl::get_location_id()] = num_g500_edges;

  stapl::scan(v, v, stapl::plus<std::size_t>(), true);

  return v[stapl::get_location_id()];
}

//////////////////////////////////////////////////////////////////////
/// @brief Convert a raw array of edges to an array_view over an array.
///
/// @param num_edges The total number of edges generated on all locations
/// @param num_g500_edges The number of edges generated on this location
/// @param edges A raw array of edges where edges[i] is a source and edges[i+1]
///   is its destination.
///
/// @todo This creates a balanced distribution of the edges, even though the
///       the generator gave a slightly imbalanced distribution. This generates
///       some traffic to neighboring locations. Ideally, we can construct
///       a stapl::array out of the raw array.
//////////////////////////////////////////////////////////////////////
stapl::array_view<stapl::array<std::pair<std::size_t, std::size_t>>>
make_edge_list_view(std::size_t num_edges, int64_t num_g500_edges,
                    int64_t* edges)
{
  auto* edge_list = new stapl::array<std::pair<std::size_t, std::size_t>>(
    num_edges
  );

  const std::size_t offset = local_edge_list_offset(num_g500_edges);

  for (std::size_t i = 0; i < static_cast<std::size_t>(num_g500_edges); ++i)
  {
    std::size_t s = edges[2*i+0];
    std::size_t t = edges[2*i+1];

    auto edge = std::make_pair(s, t);
    edge_list->set_element(offset+i, edge);
  }

  // Needed to ensure all edges are set
  stapl::rmi_fence();

  return {edge_list};
}

/*
 * This function calls the graph500 benchmark MPI implementation
 * of the input graph generator, to get the same graph as the
 * benchmark reference (the seeds are the same).
 * Each location generates a subset of the (permuted) vertices,
 * therefore, we need the add-edge methods to async to the correct
 * location. Also, the list is directed (to save space), so we need
 * to add both edges (fwd and rev).
 */
template<typename G>
void generate_benchmark_input(G* g, size_t nv, size_t nedges, size_t SCALE,
                              bool remove_multiedges = false)
{
  uint64_t seed1 = 2, seed2 = 3;
  const double initiator[4] = {.57, .19, .19, .05};
  int64_t* edges = 0;
  int64_t  nedges_64 = nedges;

  stapl::graph_view<G> graph_vw(*g);

  /* Make the raw graph edges. */
  // make_graph(SCALE, nedges_64, seed1, seed2, initiator,
  //            &nedges_64, &edges);
  bool b = stapl::external_call(make_graph, SCALE, nedges_64, seed1, seed2,
                                initiator, &nedges_64, &edges);

  stapl::counter<stapl::default_timer> add_edges_time;
  add_edges_time.start();

  const int64_t num_generated_edges = b ? nedges_64 : 0;

  auto edge_view = make_edge_list_view(nedges, num_generated_edges, edges);
  free(edges);
  stapl::transform_edge_list(edge_view, graph_vw, true);

  try_commit(*g);

  double ae_time = add_edges_time.stop();
  add_edges_time.reset();
  add_edges_time.start();

  if (remove_multiedges) {
    try_uncommit(*g);
    g->remove_duplicate_edges();
    try_commit(*g);
  }

  g->sort_edges();

  double es_time = add_edges_time.stop();

  stapl::do_once([ae_time, es_time](void)->void
                 { std::cout << "pGraph Construction Time: " << ae_time
                             << "\npGraph Edge Sort Time: " << es_time
                             << std::endl; });
}


template<typename AlgoHelper>
typename stapl::graph_view<typename AlgoHelper::graph_t>
graph500_benchmark_generator(size_t nv, size_t ef, size_t SCALE,
                             bool remove_multiedges = false)
{
  typedef typename AlgoHelper::graph_t graph_t;
  graph_t* pgraph = new graph_t(nv);

  // call the benchmark to generate input edges, then populate graph with edges.
  generate_benchmark_input(pgraph, nv, nv*ef, SCALE, remove_multiedges);

  // make sure graph is populated.
  size_t nverts = pgraph->num_vertices();
  size_t nedges = pgraph->num_edges();
  stapl::do_once([nverts, nedges]
                 { std::cout << "Num vertices: " << nverts
                             << "\nNum edges: "  << nedges
                             << std::endl; });
  stapl::rmi_fence();

  return stapl::graph_view<graph_t>(pgraph);
}


template<typename GView>
auto create_level_machine_helper(GView& vgraph)
  -> decltype(create_level_machine(vgraph, 0, 0))
{
  stapl::counter<stapl::default_timer> t;
  size_t total_cut_edges = count_cut_edges(vgraph);

  t.start();
  auto hgraph = create_level_machine(vgraph);
  double hgraph_time = t.stop();

  size_t reduced_cut_edges = count_cut_edges(vgraph);

  stapl::do_once([hgraph_time, reduced_cut_edges, total_cut_edges]
                 (void)->void
                 { std::cout << "hgraph construction time= " << hgraph_time
                             << "\ncut edges: " << reduced_cut_edges << " / "
                             << total_cut_edges << "\nCut-edge reduction %: "
                             << 100 - (reduced_cut_edges*100.0)/total_cut_edges
                             << std::endl; });
  return hgraph;
}


template<typename GView>
auto create_level2_machine_helper(GView& vgraph)
  -> decltype(create_level2_machine(vgraph, 0, 0))
{
  stapl::counter<stapl::default_timer> t;
  size_t total_cut_edges = count_cut_edges(vgraph);

  t.start();
  auto hgraph = create_level2_machine(vgraph);
  double hgraph_time = t.stop();

  size_t reduced_cut_edges = count_cut_edges(vgraph);

  stapl::do_once([hgraph_time, reduced_cut_edges, total_cut_edges]
                 (void)->void
                 { std::cout << "h2graph construction time= " << hgraph_time
                             << "\ncut edges: " << reduced_cut_edges << " / "
                             << total_cut_edges << "\nCut-edge reduction %: "
                             << 100 - (reduced_cut_edges*100.0)/total_cut_edges
                             << std::endl; });
  return hgraph;
}


template<typename GView>
auto create_level_hubs_helper(GView& vgraph, size_t hub_sz)
  -> decltype(create_level_hubs(vgraph, 0, 0))
{
  stapl::counter<stapl::default_timer> t;
  size_t total_cut_edges = count_cut_edges(vgraph);

  t.start();
  auto hgraph = create_level_hubs(vgraph, hub_sz);
  double hubs_time = t.stop();

  size_t reduced_cut_edges = count_cut_edges(vgraph);

  stapl::do_once([hubs_time, reduced_cut_edges, total_cut_edges]
                 (void)->void
                 { std::cout << "hubs graph construction time= " << hubs_time
                             << "\ncut edges: " << reduced_cut_edges << " / "
                             << total_cut_edges << "\nCut-edge reduction %: "
                             << 100-(reduced_cut_edges*100.0)/(total_cut_edges)
                             << std::endl; });
  return hgraph;
}


template<typename AlgoHelper, typename... G>
void run_algo(size_t NEXP, size_t nedges, AlgoHelper&& algo_helper,
              G&&... graph_views)
{
  stapl::counter<stapl::default_timer> t;
  double elapsed = 0.0;
  std::vector<double> t_av;

  for (size_t i=0; i<NEXP; ++i)
  {
    // call and time the Algo.
    t.reset(); t.start();
    size_t levels = algo_helper(graph_views...);
    elapsed = t.stop();
    t_av.push_back(elapsed);

    stapl::do_once([levels](void)->void
                   { std::cout << "levels= " << levels << "\n" << std::endl; });

    stapl::rmi_fence();
  }

  std::string s = algo_helper.name();
  const double avg_time = compute_stats(s, t_av);
  stapl::do_once([nedges, avg_time](void)->void
                 { std::cout << ">> Performance was: "
                             << (((double(nedges)/2.0)/avg_time)/1000000.0)
                             << " MTEPS.\n" << std::endl; });
  stapl::rmi_fence();
}

#endif

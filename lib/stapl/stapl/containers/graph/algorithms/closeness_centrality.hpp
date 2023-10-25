/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_CLOSENESS_CENTRALITY_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_CLOSENESS_CENTRALITY_HPP

#include <stapl/containers/graph/algorithms/breadth_first_search.hpp>

namespace stapl {

namespace closeness_centrality_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to compute the Farness of each vertex.
///
/// Farness(v) = Sum(d(u,v)), where d(u,v) is the shortest-distance
/// from every vertex u in the graph to the given vertex v.
/// @note In our implementation, we temporarily store the farness in the
/// same field as closeness to save space.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct farness_add_wf
{
  typedef void result_type;
  template<typename Vertex>
  result_type operator()(Vertex v) const
  { v.property().closeness(v.property().closeness()+v.property().level()); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to compute the Closeness of each vertex.
///
/// Closeness(v) = 1 / Farness(v)
///              = 1 / Sum(d(u,v)), where d(u,v) is the shortest-distance
///                from every vertex u in the graph to the given vertex v.
/// @note In our implementation, we temporarily store the farness in the
/// same field as closeness to save space, so computing the closeness
/// simply involves inverting the currently-stored value (farness).
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct compute_closeness_wf
{
  typedef void result_type;
  template<typename Vertex>
  result_type operator()(Vertex v) const
  {
    if (v.property().closeness() != 0)
      v.property().closeness(1.0 / v.property().closeness());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to initialize the Closeness of each vertex.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct init_closeness_wf
{
  typedef void result_type;
  template<typename Vertex>
  result_type operator()(Vertex v) const
  { v.property().closeness(0.0); }
};

}; // namespace closeness_centrality_impl;


//////////////////////////////////////////////////////////////////////
/// @brief Parallel Closeness Centrality Algorithm.
///
/// Calculates the closeness-centrality of each vertex in a graph.
/// Closeness(v) = 1 / Farness(v)
///              = 1 / Sum(d(u,v)), where d(u,v) is the shortest-distance
///                from every vertex u in the graph to the given vertex v.
/// @param policy A policy for execution
/// @param graph The @ref graph_view over the input graph.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Policy, typename GView>
void closeness_centrality(Policy&& policy, GView& graph)
{
  using namespace closeness_centrality_impl;
  map_func(init_closeness_wf(), graph);
  size_t size = graph.size();
  for (size_t i=0; i<size; ++i) {
    // Compute shortest-distance from this new source.
    breadth_first_search(policy, graph, i);
    // Add the previously computed values to farness.
    map_func(farness_add_wf(), graph);
  }
  // Calculate the closeness by inverting the farness.
  map_func(compute_closeness_wf(), graph);
}

} // namespace stapl
#endif

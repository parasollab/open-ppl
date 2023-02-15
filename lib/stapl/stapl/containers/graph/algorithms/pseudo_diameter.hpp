/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PSEUDO_DIAMETER_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PSEUDO_DIAMETER_HPP

#include <stapl/containers/graph/algorithms/breadth_first_search.hpp>

namespace stapl {

namespace pseudo_diameter_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to extract the out-degree of a vertex.
///
/// @tparam VD Type of the vertex-descriptor.
/// @return A pair with the descriptor and the degree of a vertex.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename VD>
struct degree_wf
{
  typedef std::pair<VD, size_t> result_type;
  template<typename Vertex>
  result_type operator()(Vertex v) const
  { return std::make_pair(v.descriptor(), v.size()); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to extract the BFS-level of a vertex.
///
/// @tparam VD Type of the vertex-descriptor.
/// @return A pair with the descriptor and the BFS-level of a vertex.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename VD>
struct level_wf
{
  typedef std::pair<VD, size_t> result_type;
  template<typename Vertex>
  result_type operator()(Vertex v) const
  { return std::make_pair(v.descriptor(), v.property().level()); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to find the max of two pairs based on the second value.
///
/// @tparam VD Type of the vertex-descriptor.
/// @return The pair with the greater of the two second values.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename VD>
struct max_value_wf
{
  typedef std::pair<VD, size_t> result_type;
  template<typename T>
  result_type operator()(T const& a, T const& b) const
  {
    if (a.second >= b.second)
      return a;
    else return b;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper to compute the vertex with the maximum out-degree.
///
/// @param graph The input @ref graph_view.
/// @return The descriptor of the first vertex with the highest out-degree.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename GView>
typename GView::vertex_descriptor select_initial_source(GView& graph)
{
  // pick a source with the largest degree, to guarantee maximum connectedness.
  auto x = map_reduce(degree_wf<typename GView::vertex_descriptor>(),
                      max_value_wf<typename GView::vertex_descriptor>(),
                      graph);
  return x.first;
}

//////////////////////////////////////////////////////////////////////
/// @brief Helper to compute the vertex with the maximum BFS-level.
///
/// @param graph The input @ref graph_view.
/// @return A pair with the descriptor and BFS-level of the first
/// vertex with the highest BFS-level.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename GView>
std::pair<typename GView::vertex_descriptor, size_t>
select_next_source(GView& graph)
{
  // pick a source with the farthest level from the last BFS.
  auto x = map_reduce(level_wf<typename GView::vertex_descriptor>(),
                      max_value_wf<typename GView::vertex_descriptor>(),
                      graph);
  return x;
}

}; // namespace pseudo_diameter_impl;


//////////////////////////////////////////////////////////////////////
/// @brief Parallel Pseudo-Diameter Algorithm.
///
/// Calculates the pseudo-diameter of a graph, using a chain of BFS
/// traversals.
/// @param policy A policy for execution
/// @param graph The @ref graph_view over the input graph.
/// @return The pseudo-diameter of the graph.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Policy, typename GView>
size_t pseudo_diameter(Policy&& policy, GView& graph)
{
  using namespace pseudo_diameter_impl;
  auto source = select_initial_source(graph);

  size_t final_pseudo_diameter = 0;
  size_t pseudo_diameter = 0;

  do {
    final_pseudo_diameter = pseudo_diameter;
    // find new diameter from this new source.
    breadth_first_search(policy, graph, source);

    // pick a new source farthest from the previous source.
    const auto x = select_next_source(graph);
    source = x.first;
    pseudo_diameter = x.second;
  } while (final_pseudo_diameter < pseudo_diameter);

  return final_pseudo_diameter - 1;
}

} // namespace stapl
#endif

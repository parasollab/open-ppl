/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_DISJOINTED_COMPLETE_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_DISJOINTED_COMPLETE_HPP

#include <stapl/containers/graph/generators/generator.hpp>
#include <stapl/containers/partitions/balanced.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which adds edges to form a graph of complete subgraphs.
//////////////////////////////////////////////////////////////////////
class disjointed_complete_neighbors
{
  using partition_type = balanced_partition<indexed_domain<std::size_t>>;
  partition_type m_part;

public:
  using result_type = void;

  //////////////////////////////////////////////////////////////////////
  /// @param num_components Number of disconnected components
  /// @param components_size Size of each individual component
  //////////////////////////////////////////////////////////////////////
  disjointed_complete_neighbors(size_t num_components, size_t component_size)
    : m_part({0, num_components*component_size-1}, num_components)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    const std::size_t me = v.descriptor();

    // Get domain of all vertices in the same component as me
    auto dom = m_part[m_part.find(v.descriptor())];

    // For directed graphs, add an edge to every vertex in the same component
    // For undirected graphs, only add an edge (u,v) if u < v
    const std::size_t start = view.is_directed() ? dom.first() : me+1;

    for (std::size_t i = start; i <= dom.last(); ++i) {
      // Don't add self edges
      if (i == me)
        continue;

      view.add_edge_async(me, i);
    }
  }

  void define_type(typer& t)
  {
    t.member(m_part);
  }
};

}


//////////////////////////////////////////////////////////////////////
/// @brief Generates several complete graphs which are disconnected.
///
/// The generated graph will have num_components*component_size vertices.
/// If vertex v and u are in the same component, then the edge (v,u) exists.
/// Otherwise, (v,u) does not exist.
///
/// This function mutates the input graph.
///
/// @param g A view over the graph to generate.
/// @param num_components Number of disconnected components
/// @param components_size Size of each individual component
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_complete(GraphView& g,
                        std::size_t num_components,
                        std::size_t component_size)
{
  using ef_t = detail::disjointed_complete_neighbors;
  const std::size_t n = num_components * component_size;

  return make_generator<GraphView, ef_t>(
    g, n, ef_t(num_components, component_size))();
}

//////////////////////////////////////////////////////////////////////
/// @brief Generates several complete graphs which are disconnected.
///
/// The generated graph will have num_components*component_size vertices.
/// If vertex v and u are in the same component, then the edge (v,u) exists.
/// Otherwise, (v,u) does not exist.
///
/// The returned view owns its underlying container.
///
/// @param num_components Number of disconnected components
/// @param components_size Size of each individual component
/// @return A view over the generated graph.
/// @b Example
/// @snippet disjoint_complete.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_disjointed_complete(std::size_t num_components,
                                   std::size_t component_size)
{
  using ef_t = detail::disjointed_complete_neighbors;
  const std::size_t n = num_components * component_size;

  return make_generator<GraphView, ef_t>(
    n, ef_t(num_components, component_size))();
}

} // namespace generators

} // namespace stapl

#endif

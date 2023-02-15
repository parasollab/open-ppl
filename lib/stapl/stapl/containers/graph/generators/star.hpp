/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_STAR_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_STAR_HPP

#include <stapl/containers/graph/generators/generator.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which adds edges to form a star of the given dimensions.
/// @see make_star
//////////////////////////////////////////////////////////////////////
struct star_neighbors
{
  std::size_t m_root;
  bool m_bidirectional;

  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param root The descriptor of the vertex that will be the root of the star
  /// @param bidirectional Flag indicating that edges should be added in both
  ///                      directions.
  //////////////////////////////////////////////////////////////////////
  star_neighbors(std::size_t root, bool bidirectional)
    : m_root(root), m_bidirectional(bidirectional)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    if (v.descriptor() != m_root)
    {
      view.add_edge_async(v.descriptor(), m_root);

      if (view.is_directed() && m_bidirectional)
        view.add_edge_async(m_root, v.descriptor());
    }
  }

  void define_type(typer& t)
  {
    t.member(m_root);
    t.member(m_bidirectional);
  }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Generates a star graph of size n, with root r.
///
/// The star graph will have the property that all of the vertices have an edge
/// to a single particular vertex, designated the root. If the graph generated
/// is not bidirectional, all non-root vertices will have a directed edge
/// to the root and the root will have no outgoing edges.
///
/// This function mutates the input graph.
///
/// @param g A view over the graph to generate.
/// @param n Size of the star.
/// @param r The descriptor of the vertex that is the root.
/// @param bidirectional Edges should be added in both directions
///
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_star(GraphView& g, std::size_t n, std::size_t r,
                    bool bidirectional = true)
{
  typedef typename detail::star_neighbors ef_t;

  return make_generator<GraphView, ef_t>(g, n, ef_t(r, bidirectional))();
}


//////////////////////////////////////////////////////////////////////
/// @brief Generates a star graph of size n, with root r.
///
/// The star graph will have the property that all of the vertices have an edge
/// to a single particular vertex, designated the root. If the graph generated
/// is not bidirectional, all non-root vertices will have a directed edge
/// to the root and the root will have no outgoing edges.
///
/// The returned view owns its underlying container.
///
/// @param n Size of the star.
/// @param r The descriptor of the vertex that is the root.
/// @param bidirectional Edges should be added in both directions
///
/// @return The original view, now containing the generated graph.
///
/// @b Example
/// @snippet star.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_star(std::size_t n, std::size_t r, bool bidirectional = true)
{
  typedef typename detail::star_neighbors ef_t;

  return make_generator<GraphView, ef_t>(n, ef_t(r, bidirectional))();
}

} // namespace generators

} // namespace stapl

#endif

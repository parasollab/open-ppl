/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_COMPLETE_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_COMPLETE_HPP

#include <stapl/containers/graph/generators/generator.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which adds edges to form a complete graph.
//////////////////////////////////////////////////////////////////////
struct complete_neighbors
{
  size_t m_n;
  bool m_bidirectional;

  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param n Number of vertices in the output graph.
  /// @param bidirectional True to add back-edges in a directed graph, false
  ///   for forward edges only.
  //////////////////////////////////////////////////////////////////////
  complete_neighbors(size_t n, bool bidirectional)
    : m_n(n), m_bidirectional(bidirectional)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    if (v.descriptor() < m_n) {
      for (size_t i = 0; i < m_n; ++i)
        if (i != v.descriptor()) {
          view.add_edge_async(v.descriptor(), i);
          if (view.is_directed() && m_bidirectional)
            view.add_edge_async(i, v.descriptor());
        }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_n);
    t.member(m_bidirectional);
  }
};

}


//////////////////////////////////////////////////////////////////////
/// @brief Generate a complete graph.
///
/// The generated graph will have n vertices and all vertices will
/// have an edge to all of the other vertices.
///
/// This function mutates the input graph.
///
/// @param g A view over the graph to generate.
/// @param n Number of vertices in the output graph.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_complete(GraphView& g, size_t n, bool bidirectional=true)
{
  typedef typename detail::complete_neighbors ef_t;
  return make_generator<GraphView, ef_t>(g, n, ef_t(n, bidirectional))();
}

//////////////////////////////////////////////////////////////////////
/// @brief Generate a complete graph.
///
/// The generated graph will have n vertices and all vertices will
/// have an edge to all of the other vertices.
///
/// The returned view owns its underlying container.
///
/// @param n Number of vertices in the output graph.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return A view over the generated graph.
///
/// ! https://i.imgur.com/cOFsYbf.png
///
/// @b Example
/// @snippet complete.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_complete(size_t n, bool bidirectional=true)
{
  typedef typename detail::complete_neighbors ef_t;
  return make_generator<GraphView, ef_t>(n, ef_t(n, bidirectional))();
}

} // namespace generators

} // namespace stapl

#endif

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_BINARY_TREE_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_BINARY_TREE_HPP

#include <stapl/containers/graph/generators/generator.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which adds edges to generate a binary tree.
//////////////////////////////////////////////////////////////////////
struct binary_tree_neighbors
{
  bool m_bidirectional;

  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param bidirectional True to add back-edges in a directed graph, false
  ///   for forward edges only.
  //////////////////////////////////////////////////////////////////////
  binary_tree_neighbors(bool bidirectional)
    : m_bidirectional(bidirectional)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    size_t i = v.descriptor();
    size_t t1 = 2*i+1;
    size_t t2 = 2*i+2;

    if (t1 < view.size())
      view.add_edge_async(i, t1);
    if (t2 < view.size())
      view.add_edge_async(i, t2);

    if (view.is_directed() && m_bidirectional) {
      if (t1 < view.size())
        view.add_edge_async(t1, i);
      if (t2 < view.size())
        view.add_edge_async(t2, i);
    }
  }

  void define_type(typer& t)
  { t.member(m_bidirectional); }
};

}


//////////////////////////////////////////////////////////////////////
/// @brief Generates a binary tree with n vertices.
///
///        This function mutates the input graph.
///
/// @param g A view over the graph to generate.
/// @param n Number of vertices in the generated graph.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_binary_tree(GraphView& g, size_t n, bool bidirectional=true)
{
  typedef typename detail::binary_tree_neighbors ef_t;
  return make_generator<GraphView, ef_t>(g, n, ef_t(bidirectional))();
}

//////////////////////////////////////////////////////////////////////
/// @brief Generates a binary tree with n vertices.
///
///        The returned view owns its underlying container.
///
/// @param n Number of vertices in the generated graph.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return A view over the generated graph.
/// @see make_binary_tree
/// @attention sgl-reference
///
/// ! https://i.imgur.com/lZSDjES.png
///
/// @b Example
/// @snippet binary_tree.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_binary_tree(size_t n, bool bidirectional=true)
{
  typedef typename detail::binary_tree_neighbors ef_t;
  return make_generator<GraphView, ef_t>(n, ef_t(bidirectional))();
}

} // namespace generators

} // namespace stapl

#endif

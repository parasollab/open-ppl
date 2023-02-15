/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_LISTS_TREE_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_LISTS_TREE_HPP

#include <stapl/containers/graph/generators/generator.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which adds edges to form a tree-of-lists graph of the given
///   size.
//////////////////////////////////////////////////////////////////////
struct lists_tree_neighbors
{
  size_t m_x;
  size_t m_y;
  bool m_bidirectional;

  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param nx Number of branches in the tree.
  /// @param ny Number of vertices in each list.
  /// @param bidirectional True for adding back-edges, False for forward only.
  //////////////////////////////////////////////////////////////////////
  lists_tree_neighbors(size_t nx, size_t ny, bool bidirectional)
    : m_x(nx), m_y(ny), m_bidirectional(bidirectional)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    size_t i = v.descriptor();
    for (size_t j = 0; j < m_y; ++j) {
      size_t s = (i*m_y) + j;
      size_t t = (((i+1))*m_y) + j;

      if (t < m_x*m_y) {
        view.add_edge_async(s, t);
        if (view.is_directed() && m_bidirectional)
          view.add_edge_async(t, s);
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_x);
    t.member(m_y);
    t.member(m_bidirectional);
  }
};

}


//////////////////////////////////////////////////////////////////////
/// @brief Generate a tree-of-lists graph.
///
/// The generated graph is a tree of x branches, each
/// of which is a list containing y vertices.
///
/// This function mutates the input graph.
///
/// @param g A view over the graph to generate.
/// @param nx Number of branches in the tree.
/// @param ny Number of vertices in each list.
/// @param bidirectional True for adding back-edges, False for forward only.
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_lists_tree(GraphView& g, size_t nx, size_t ny,
                          bool bidirectional=true)
{
  typedef typename detail::lists_tree_neighbors ef_t;
  GraphView vw =
    make_generator<GraphView, ef_t>(g, nx*ny, ef_t(nx, ny, bidirectional))();

  if (get_location_id() == 0)
    for (size_t i = 0; i < ny-1; ++i)
      vw.add_edge_async(i, i+1);

  return vw;
}

//////////////////////////////////////////////////////////////////////
/// @brief Generate a tree-of-lists graph.
///
/// The generated graph is a tree of x branches, each
/// of which is a list containing y vertices.
///
/// The returned view owns its underlying container.
///
/// @param nx Number of branches in the tree.
/// @param ny Number of vertices in each list.
/// @param bidirectional True for adding back-edges, False for forward only.
/// @return A view over the generated graph.
///
/// @b Example
/// @snippet lists_tree.cc Example
/////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_lists_tree(size_t nx, size_t ny, bool bidirectional=true)
{
  typedef typename detail::lists_tree_neighbors ef_t;
  GraphView vw =
    make_generator<GraphView, ef_t>(nx*ny, ef_t(nx, ny, bidirectional))();

  if (get_location_id() == 0)
    for (size_t i = 0; i < ny-1; ++i)
      vw.add_edge_async(i, i+1);

  return vw;
}

} // namespace generators

} // namespace stapl

#endif

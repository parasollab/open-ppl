/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_LIST_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_LIST_HPP

#include <stapl/containers/graph/generators/generator.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which adds edges to form a list graph.
//////////////////////////////////////////////////////////////////////
struct list_neighbors
{
  size_t m_n;
  size_t m_start;
  bool   m_bidirectional;

  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param n Size of the graph.
  /// @param start First vertex id of the list.
  /// @param bidirectional True for adding back-edges, False for forward only.
  //////////////////////////////////////////////////////////////////////
  list_neighbors(size_t n, size_t start, bool bidirectional)
    : m_n(n), m_start(start), m_bidirectional(bidirectional)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    if (v.descriptor() < m_start+m_n-1 && v.descriptor() >= m_start) {
      view.add_edge_async(v.descriptor(), v.descriptor()+1);
      if (view.is_directed() && m_bidirectional)
        view.add_edge_async(v.descriptor()+1, v.descriptor());
    }
  }

  void define_type(typer& t)
  {
    t.member(m_n);
    t.member(m_start);
    t.member(m_bidirectional);
  }
};

}


//////////////////////////////////////////////////////////////////////
/// @brief Generate a list graph.
///
/// The generated list will contain n vertices, each with an edge to its
/// next element in the list (except for the last).
///
/// This function mutates the input graph.
///
/// @param g A view over the graph to generate.
/// @param n The number of nodes in the list.
/// @param bidirectional True for adding back-edges, False for forward only.
/// @return The original view, now containing the generated graph.
/////////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_list(GraphView& g, size_t n, bool bidirectional=true)
{
  typedef typename detail::list_neighbors     ef_t;
  return make_generator<GraphView, ef_t>(g, n, ef_t(n, 0, bidirectional))();
}


//////////////////////////////////////////////////////////////////////
/// @brief Generate a list graph.
///
/// The generated list will contain n vertices, each with an edge to its
/// next element in the list (except for the last).
///
/// The returned view owns its underlying container.
///
/// @param n The number of nodes in the list.
/// @param bidirectional True for adding back-edges, False for forward only.
/// @return A view over the generated graph.
///
/// @b Example
/// @snippet list.cc Example
/////////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_list(size_t n, bool bidirectional=true)
{
  typedef typename detail::list_neighbors     ef_t;
  return make_generator<GraphView, ef_t>(n, ef_t(n, 0, bidirectional))();
}

} // namespace generators

} // namespace stapl

#endif

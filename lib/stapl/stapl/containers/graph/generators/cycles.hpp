/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_CYCLES_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_CYCLES_HPP

#include <stapl/containers/graph/generators/generator.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which adds edges to form a chain/collection of cycles.
//////////////////////////////////////////////////////////////////////
struct cycle_neighbors
{
  size_t m_cycle_count, m_cycle_size, m_size;
  bool m_connected, m_bidirectional;

  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param cycle_count Number of cycles in the generated graph.
  /// @param cycle_size Number of vertices in each cycle.
  /// @param connected True if each cycle is connected to the next one.
  /// @param bidirectional True to add back-edges in a directed graph, false
  ///   for forward edges only.
  //////////////////////////////////////////////////////////////////////
  cycle_neighbors(size_t cycle_count, size_t cycle_size, bool connected,
                  bool bidirectional)
    : m_cycle_count(cycle_count), m_cycle_size(cycle_size),
      m_size(cycle_count*cycle_size),
      m_connected(connected), m_bidirectional(bidirectional)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    size_t j = v.descriptor() + 1;
    bool chain_complete = false;

    if (j % m_cycle_size == 0) {
      chain_complete = true;
      size_t j2 = j-m_cycle_size;
      view.add_edge_async(v.descriptor(), j2);
      if (view.is_directed() && m_bidirectional)
        view.add_edge_async(j2, v.descriptor());
    }

    if (!(chain_complete && !m_connected)) {
      if (j < m_size) {
        view.add_edge_async(v.descriptor(), j);
        if (view.is_directed() && m_bidirectional)
          view.add_edge_async(j, v.descriptor());
      }
    }

  }

  void define_type(typer& t)
  {
    t.member(m_cycle_count);
    t.member(m_cycle_size);
    t.member(m_size);
    t.member(m_connected);
    t.member(m_bidirectional);
  }
};

}


//////////////////////////////////////////////////////////////////////
/// @brief Generates a graph containing many linked cycles.
///
/// The graph will contain cycle_count cycles, each of which has
/// cycle_size vertices. The largest vertex of each cycle is connected to the
/// smallest vertex of the next cycle.
///
/// Results in cycle_count SCCs and one weakly-connected component in a directed
/// graph. Results in a single CC in an undirected graph.
///
/// This function mutates the input graph.
///
/// @param g A view over the graph to generate.
/// @param cycle_count Number of cycles in the generated graph.
/// @param cycle_size Number of vertices in each cycle.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_cycle_chain(GraphView& g, size_t cycle_count, size_t cycle_size,
                           bool bidirectional=true)
{
  typedef typename detail::cycle_neighbors ef_t;
  return make_generator<GraphView, ef_t>(g, cycle_count*cycle_size,
                                         ef_t(cycle_count, cycle_size, true,
                                              bidirectional))();
}

//////////////////////////////////////////////////////////////////////
/// @brief Generates a graph containing many linked cycles.
///
/// The graph will contain cycle_count cycles, each of which has
/// cycle_size vertices. The largest vertex of each cycle is connected to the
/// smallest vertex of the next cycle.
///
/// Results in cycle_count SCCs and one weakly-connected component in a directed
/// graph. Results in a single CC in an undirected graph.
///
/// The returned view owns its underlying container.
///
/// @param cycle_count Number of cycles in the generated graph.
/// @param cycle_size Number of vertices in each cycle.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return A view over the generated graph.
///
/// @b Example
/// @snippet cycle_chain.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_cycle_chain(size_t cycle_count, size_t cycle_size,
                           bool bidirectional=true)
{
  typedef typename detail::cycle_neighbors ef_t;
  return make_generator<GraphView, ef_t>(cycle_count*cycle_size,
                                         ef_t(cycle_count, cycle_size, true,
                                              bidirectional))();
}


//////////////////////////////////////////////////////////////////////
/// @brief Generates a graph containing many unconnected cycles.
///
/// The generated graph will contain cycle_count cycles, each of which has
/// cycle_size vertices. The cycles are not connected.
///
/// Results in cycle_count SCCs/WCCs in a directed
/// graph. Results in cycle_count CCs in an undirected graph.
///
/// This function mutates the input graph.
///
/// @param g A view over the graph to generate.
/// @param cycle_count Number of cycles in the generated graph.
/// @param cycle_size Number of vertices in each cycle.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_unconnected_cycles(GraphView& g, size_t cycle_count,
                                  size_t cycle_size,
                                  bool bidirectional=true)
{
  typedef typename detail::cycle_neighbors ef_t;
  return make_generator<GraphView, ef_t>(g, cycle_count*cycle_size,
                                         ef_t(cycle_count, cycle_size, false,
                                              bidirectional))();
}

//////////////////////////////////////////////////////////////////////
/// @brief Generates a graph containing many unconnected cycles.
///
/// The generated graph will contain cycle_count cycles, each of which has
/// cycle_size vertices. The cycles are not connected.
///
/// Results in cycle_count SCCs/WCCs in a directed
/// graph. Results in cycle_count CCs in an undirected graph.
///
/// The returned view owns its underlying container.
///
/// @param cycle_count Number of cycles in the generated graph.
/// @param cycle_size Number of vertices in each cycle.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return A view over the generated graph.
///
/// @b Example
/// @snippet unconnected_cycles.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_unconnected_cycles(size_t cycle_count, size_t cycle_size,
                           bool bidirectional=true)
{
  typedef typename detail::cycle_neighbors ef_t;
  return make_generator<GraphView, ef_t>(cycle_count*cycle_size,
                                         ef_t(cycle_count, cycle_size, false,
                                              bidirectional))();
}


} // namespace generators

} // namespace stapl

#endif

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_RANDOM_NEIGHBORHOOD_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_RANDOM_NEIGHBORHOOD_HPP

#include <stapl/containers/graph/generators/generator.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Work function which adds edges to the given graph to form a random
///   neighborhood.
//////////////////////////////////////////////////////////////////////
struct random_k_neighbors
  : protected rand_gen
{
  size_t m_size, m_local_size, m_ef, m_k;
  bool m_bidirectional;

  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param num_vertices Total size of graph.
  /// @param local_size Number of vertices on a single location.
  /// @param Name Description
  /// @param seed The seed for random-number generation.
  //////////////////////////////////////////////////////////////////////
  random_k_neighbors(size_t num_vertices, size_t local_size, size_t ef,
                     size_t k, bool bidirectional,
                     unsigned int seed = get_location_id())
    : rand_gen(seed), m_size(num_vertices), m_local_size(local_size),
      m_ef(ef), m_k(k), m_bidirectional(bidirectional)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the edges for the portion of the vertices assigned to location
  ///   x.
  /// @param x Location id which owns vertices where edges are added.
  /// @param view View over the graph to which the edges are added.
  ///
  /// @bug Wrapping of target vertex ids at the ends of the range is not handled
  ///   properly (-5 from vertex 0).
  /// @bug Improper use of size_t where negative numbers are expected
  ///   (tgt = (0 - tgt)).
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename V>
  void operator()(T x, V& view)
  {
    for (size_t i = 0; i < m_ef*m_local_size; ++i)
    {
      // pick a random source vertex.
      size_t src = this->rand(m_local_size) + m_local_size*x;
      // pick a random target in k neighborhood of the source.
      size_t tgt = this->rand(m_k);
      while (tgt == 0)
        tgt = this->rand(m_k);

      // choose -k neighbor with 50% probability
      if (this->rand(100) >= 50)
        tgt = (0 - tgt);
      tgt += src;
      // sanitize, within range, wrap, etc.
      tgt = tgt % m_size;

      // add the edges.
      view.add_edge_async(src, tgt);
      if (m_bidirectional)
        view.add_edge_async(tgt, src);
    }
  }

  void define_type(typer& t)
  {
    t.member(m_size);
    t.member(m_local_size);
    t.member(m_ef);
    t.member(m_k);
    t.member(m_bidirectional);
  }
};

}

//////////////////////////////////////////////////////////////////////
/// @brief Graph generator for a random neighborhood graph.
/// @tparam G Type of the graph view used to construct the graph.
///
/// @see make_random_neighborhood
//////////////////////////////////////////////////////////////////////
template<typename G>
struct random_neighborhood
  : public generator_base<random_neighborhood<G> >
{
  typedef generator_base<random_neighborhood<G> > base_type;

protected:
  size_t m_ef;
  size_t m_k;
  bool   m_bidirectional;
public:
  //////////////////////////////////////////////////////////////////////
  /// @param g View of the graph to generate
  /// @param n The number of nodes in the graph
  /// @param ef The average number of edges per vertex in the graph.
  /// @param k The farthest neighbor that a vertex may connect to (+-k).
  /// @param bidirectional True to add back-edges in a directed graph, false
  ///   for forward edges only.
  //////////////////////////////////////////////////////////////////////
  random_neighborhood(G& g, size_t n, size_t ef, size_t k,
                      bool bidirectional=true)
    : base_type(g, n), m_ef(ef), m_k(k),
      m_bidirectional(bidirectional)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param n The number of nodes in the graph
  /// @param ef The average number of edges per vertex in the graph.
  /// @param k The farthest neighbor that a vertex may connect to (+-k).
  /// @param bidirectional True to add back-edges in a directed graph, false
  ///   for forward edges only.
  //////////////////////////////////////////////////////////////////////
  random_neighborhood(size_t n, size_t ef, size_t k,
                      bool bidirectional=true)
    : base_type(n), m_ef(ef), m_k(k),
      m_bidirectional(bidirectional)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function which adds vertices to the graph.
  //////////////////////////////////////////////////////////////////////
  void add_vertices()
  {
    base_type::add_vertices();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function which adds the edges to the graph.
  ///
  /// @bug Truncating division with get_num_locations() may cause no edges to
  ///   be added for last N % get_num_locations() vertices.
  ///
  /// @see detail::random_k_neighbors
  //////////////////////////////////////////////////////////////////////
  void add_edges()
  {
    // add m_ef*m_vertices edges.
    bool bidirectional = this->graph().is_directed() && m_bidirectional;
    size_t num_vertices = this->graph().size();
    map_func(detail::random_k_neighbors(num_vertices,
                                        num_vertices/get_num_locations(),
                                        m_ef, m_k, bidirectional),
             counting_view<size_t>(get_num_locations()),
             make_repeat_view(this->graph()));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Generates a random graph with N vertices.
///
/// Each vertex
/// can connect to others in its neighborhood (+/- k
/// from a vertex's id defines the neighborhood). @c k=1 forms a circular list,
/// while @c k=N forms an Erdos-Renyi random network. Each vertex connects to
/// @c ef neighbors on average, this is not the guaranteed number of edges for
/// a particular vertex. The diameter of the generated graph decreases as @c k
/// increases.
///
/// This function mutates the input graph.
///
/// @param g A view over the graph to generate.
/// @param n Number of vertices in the generated graph.
/// @param ef Average number of outgoing edges per vertex.
/// @param k Size of the neighborhood to which each vertex can connect.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_random_neighborhood(GraphView& g, size_t n, size_t ef, size_t k,
                                   bool bidirectional=true)
{
  return random_neighborhood<GraphView>(g, n, ef, k, bidirectional)();
}


//////////////////////////////////////////////////////////////////////
/// @brief Generates a random graph with N vertices.
///
/// Each vertex
/// can connect to others in its neighborhood (+/- k
/// from a vertex's id defines the neighborhood). @c k=1 forms a circular list,
/// while @c k=N forms an Erdos-Renyi random network. Each vertex connects to
/// @c ef neighbors on average, this is not the guaranteed number of edges for
/// a particular vertex. The diameter of the generated graph decreases as @c k
/// increases.
///
/// The returned view owns its underlying container.
///
/// @param n Number of vertices in the generated graph.
/// @param ef Average number of outgoing edges per vertex.
/// @param k Size of the neighborhood to which each vertex can connect.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return A view over the generated graph.
///
/// @b Example
/// @snippet random_neighborhood.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_random_neighborhood(size_t n, size_t ef, size_t k,
                                   bool bidirectional=true)
{
  return random_neighborhood<GraphView>(n, ef, k, bidirectional)();
}


} // namespace generators

} // namespace stapl

#endif

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_ERDOS_RENYI_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_ERDOS_RENYI_HPP

#include <stapl/containers/graph/generators/generator.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/counting_view.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which adds edges to form an Erdos-Renyi random network.
//////////////////////////////////////////////////////////////////////
struct erdos_renyi_neighbors
{
  size_t   m_size;
  double   m_prob;
  bool     m_bidirectional;
  rand_gen m_rng;

  using result_type = void;

  //////////////////////////////////////////////////////////////////////
  /// @param size Number of vertices in the output graph.
  /// @param prob The probability of adding any given edge.
  /// @param bidirectional True to add back-edges in a directed graph, false
  ///   for forward edges only.
  /// @param seed The seed for random-number generation.
  //////////////////////////////////////////////////////////////////////
  erdos_renyi_neighbors(size_t size, double prob, bool bidirectional,
                        unsigned int seed = get_location_id())
    : m_size(size), m_prob(prob*100), m_bidirectional(bidirectional),
      m_rng(seed)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    // Each vertex v attempts to add edges to all vertices u where u > v.
    // The probability of adding an edge is m_prob.
    for (size_t i = v.descriptor()+1; i < m_size; i++) {
      if (m_rng.rand(100) <= m_prob) {
        view.add_edge_async(v.descriptor(), i);
        if (view.is_directed() && m_bidirectional)
          view.add_edge_async(i, v.descriptor());
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_size);
    t.member(m_prob);
    t.member(m_bidirectional);
    t.member(m_rng);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to add a random edge
//////////////////////////////////////////////////////////////////////
struct add_random_edge
{
  std::size_t m_size;
  bool m_bidirectional;
  rand_gen m_rng;

  using result_type = void;

  //////////////////////////////////////////////////////////////////////
  /// @param size Number of vertices in the output graph.
  /// @param bidirectional True to add back-edges in a directed graph, false
  ///   for forward edges only.
  /// @param seed The seed for random-number generation.
  //////////////////////////////////////////////////////////////////////
  add_random_edge(size_t size,  bool bidirectional,
                  unsigned int seed = get_location_id())
    : m_size(size), m_bidirectional(bidirectional), m_rng(seed)
  { }

  template<typename Index, typename Graph>
  void operator()(Index&&, Graph& view)
  {
    const std::size_t src = m_rng.rand(m_size);
    const std::size_t tgt = m_rng.rand(m_size);

    view.add_edge_async(src, tgt);

    if (view.is_directed() && m_bidirectional)
      view.add_edge_async(tgt, src);
  }

  void define_type(typer& t)
  {
    t.member(m_size);
    t.member(m_bidirectional);
    t.member(m_rng);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Graph generator for a Erdos-Renyi type M graph
/// @tparam G Type of the graph view used to construct the graph.
//////////////////////////////////////////////////////////////////////
template <typename G>
struct erdos_renyi_m_generator
  : public generator_base<erdos_renyi_m_generator<G>>
{
  using base_type = generator_base<erdos_renyi_m_generator<G>>;

protected:
  std::size_t m_n;
  std::size_t m_m;
  bool m_bidirectional;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param size The number of vertices
  /// @param m The number of edges to generate
  /// @param bidirectional Whether to add back edges
  //////////////////////////////////////////////////////////////////////
  erdos_renyi_m_generator(std::size_t size, std::size_t m, bool bidirectional)
    : base_type(size), m_n(size), m_m(m), m_bidirectional(bidirectional)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param g View over the graph to generate.
  /// @param size The number of vertices
  /// @param m The number of edges to generate
  /// @param bidirectional Whether to add back edges
  //////////////////////////////////////////////////////////////////////
  erdos_renyi_m_generator(G& g, std::size_t size, std::size_t m,
                          bool bidirectional)
    : base_type(g, size), m_n(size), m_m(m), m_bidirectional(bidirectional)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function which is called to add edges to the graph.
  //////////////////////////////////////////////////////////////////////
  void add_edges()
  {
    stapl_assert(this->graph().size() == m_n,
      "erdos_renyi_m_generator::add_edges(): incorrect number of vertices");

    map_func(
      add_random_edge{m_n, m_bidirectional},
      counting_view<std::size_t>(m_m),
      make_repeat_view(this->graph())
    );
  }
};

}


//////////////////////////////////////////////////////////////////////
/// @brief Generates a G(n,p) Erdos-Renyi random network.
///
/// The generated Erdos-Renyi random network has the property that
/// the probability of adding an edge u -> v is p, where u > v.
///
/// This function mutates the input graph.
///
/// @param g A view over the graph to generate.
/// @param size Number of vertices in the output graph.
/// @param prob The probability of adding any given edge.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_erdos_renyi(GraphView& g, size_t size, double prob,
                           bool bidirectional=true)
{
  typedef typename detail::erdos_renyi_neighbors ef_t;
  return make_generator<GraphView, ef_t>(g, size, ef_t(size, prob,
                                                       bidirectional))();
}


//////////////////////////////////////////////////////////////////////
/// @brief Generates a G(n,p) Erdos-Renyi random network.
///
/// The generated Erdos-Renyi random network has the property that
/// the probability of adding an edge u -> v is p, where u > v.
///
/// The returned view owns its underlying container.
///
/// @param size Number of vertices in the output graph.
/// @param prob The probability of adding any given edge.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return A view over the generated graph.
///
/// @b Example
/// @snippet erdos_renyi.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_erdos_renyi(size_t size, double prob, bool bidirectional=true)
{
  typedef typename detail::erdos_renyi_neighbors ef_t;
  return make_generator<GraphView, ef_t>(size,
                                         ef_t(size, prob, bidirectional))();
}

//////////////////////////////////////////////////////////////////////
/// @brief Generates a G(n,M) Erdos-Renyi random network.
///
/// The generated Erdos-Renyi random network has m edges, drawn
/// uniformly at random. Note that some edges may be randomly generated
/// twice, and thus there may be less than m edges for a non-multiedge
/// graph.
/// The returned view owns its underlying container.
///
/// @param size Number of vertices in the output graph.
/// @param m The number of edges in the graph
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return A view over the generated graph.
///
/// @b Example
/// @snippet erdos_renyi_m.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_erdos_renyi_m(std::size_t size, std::size_t m,
                             bool bidirectional=false)
{
  detail::erdos_renyi_m_generator<GraphView> gen{size, m, bidirectional};
  return gen();
}

//////////////////////////////////////////////////////////////////////
/// @brief Generate a G(n,M) Erdos-Renyi random network.
///
/// The generated Erdos-Renyi random network has m edges, drawn
/// uniformly at random. Note that some edges may be randomly generated
/// twice, and thus there may be less than m edges for a non-multiedge
/// graph.
///
/// The returned view owns its underlying container.
///
/// @param size Number of vertices in the output graph.
/// @param m The number of edges in the graph
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return A view over the generated graph.
///
/// @b Example
/// @snippet erdos_renyi_m.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_erdos_renyi_m(GraphView& g, std::size_t size, std::size_t m,
                             bool bidirectional=false)
{
  detail::erdos_renyi_m_generator<GraphView> gen{g, size, m, bidirectional};
  return gen();
}


} // namespace generators

} // namespace stapl

#endif

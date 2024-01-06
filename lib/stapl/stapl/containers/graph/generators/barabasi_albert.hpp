/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_BARABASI_ALBERT_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_BARABASI_ALBERT_HPP

#include <stapl/array.hpp>
#include <stapl/utility/hash.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/repeated_view.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor that generates the i'th edge that should exist in
///        the graph, giving preferential selection for lower vertex
///        descriptors.
//////////////////////////////////////////////////////////////////////
class generate_barabasi_albert_edge
{
  std::size_t m_d;

public:
  using result_type = void;

  generate_barabasi_albert_edge(const std::size_t d)
    : m_d(d)
  { }

  template<typename Edge, typename Index>
  void operator()(Edge e, Index i)
  {
    std::size_t r = 2*i+1;
    std::mt19937 gen(r);
    std::uniform_int_distribution<std::size_t> dis(0, r-1);
    do {
      r = dis(gen);
    } while (r % 2 == 1);

    const std::size_t source = i/m_d;
    const std::size_t target = r/(2*m_d);

    e.first = source;
    e.second = target;
  }

  void define_type(typer& t)
  {
    t.member(m_d);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor that adds a given edge to a graph.
//////////////////////////////////////////////////////////////////////
class add_edge
{
  bool m_add_reverse_edge;

public:
  add_edge(bool add_reverse_edge)
    : m_add_reverse_edge(add_reverse_edge)
  { }

  using result_type = void;

  template<typename Edge, typename Graph>
  void operator()(Edge e, Graph g)
  {
    g.add_edge_async(e.first, e.second);

    if (m_add_reverse_edge)
      g.add_edge_async(e.second, e.first);
  }

  void define_type(typer& t)
  {
    t.member(m_add_reverse_edge);
  }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Generate a scale-free network based on the Barabasi-Albert
///        preferential attachment model.
///
///        This implementation is based on Sanders and Schulz [^1].
///
///        Vertices with lower vertex IDs are more likely to be the target for
///        edges. Note that if add_reverse_edge is false, the outgoing edge
///        distribution is constant, as each vertex will have exactly 'd' edges.
///
///        The returned view owns its underlying container.
///
///
/// @param n Number of vertices in the generated graph.
/// @param d The number of outgoing edges to generate for each vertex
/// @return A view over the generated graph.
///
/// ! https://i.imgur.com/2nwNn5l.png
///
/// @b Example
/// @snippet binary_tree.cc Example
///
/// [^1] Scalable Generation of Scale-free Graphs. Peter Sanders, Christian
///      Schulz. arXiv:1602.07106v1. [cs.DS] 23 Feb 2016.
///      http://arxiv.org/pdf/1602.07106v1.pdf
/// @addtogroup SGLReferenceGenerators
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_barabasi_albert(const std::size_t n, const std::size_t d,
                               bool add_reverse_edge = true)
{
  using graph_type = typename view_traits<GraphView>::container;
  using edge_type = std::pair<std::size_t, std::size_t>;
  using edge_container_type = static_array<edge_type>;
  using edge_view_type = array_view<edge_container_type>;

  // Create graph and edge container
  auto* g = new graph_type(n);
  GraphView view(g);

  edge_container_type edge_container(n*d);
  edge_view_type edges(edge_container);

  // Generate random source and target for each index
  map_func(detail::generate_barabasi_albert_edge(d), edges,
    counting_view<std::size_t>(edges.size())
  );

  // Populate graph from edge container
  map_func(detail::add_edge(add_reverse_edge), edges, make_repeat_view(view));

  return view;
}

} // namespace generators

} // namespace stapl

#endif

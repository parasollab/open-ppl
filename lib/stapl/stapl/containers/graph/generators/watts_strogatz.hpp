/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_WATTS_STROGATZ_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_WATTS_STROGATZ_HPP

#include <stapl/containers/graph/generators/generator.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which connects vertices with their k-nearest neighbors to
///   form a Watts-Strogatz graph.
/// @see make_watts_strogatz
/// @see make_newman_watts_strogatz
//////////////////////////////////////////////////////////////////////
struct ws_neighbors
{
  size_t m_i, m_n;
  bool m_bidirectional;

  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param i Distance of neighbor to connect to.
  /// @param n Size of the graph.
  /// @param bidirectional True to add backedges, false otherwise.
  //////////////////////////////////////////////////////////////////////
  ws_neighbors(size_t i, size_t n, bool bidirectional)
    : m_i(i), m_n(n), m_bidirectional(bidirectional)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    const auto src = v.descriptor();
    const auto tgt = (v.descriptor()+m_i)%m_n;
    view.add_edge_async(src, tgt);

    if (m_bidirectional && view.is_directed()) {
      view.add_edge_async(tgt, src);
    }
  }

  void define_type(typer& t)
  {
    t.member(m_i);
    t.member(m_n);
    t.member(m_bidirectional);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor which rewires edges in a Watts-Strogatz graph.
/// @tparam Delete Flag which indicates if an existing edge is deleted when a
///   new one is added.
/// @see make_watts_strogatz
/// @see make_newman_watts_strogatz
/// @todo Need to fix stability of random generation to work for strong scaling,
/// for a given max processors and remove @ref watts_strogatz_stable.
//////////////////////////////////////////////////////////////////////
template<bool Delete>
class ws_rewire
  : public rand_gen
{
  double m_p;
  size_t m_n;
  bool   m_bidirectional;
  size_t m_max_proc;
  size_t m_vert_per_proc;
  size_t m_nlocs;
  size_t m_loc_id;
  size_t m_start_proc;

  template<typename View, typename Edge>
  void delete_edge(View&, Edge&&, bool, std::false_type)
  { }

  template<typename View, typename Edge>
  void delete_edge(View& view, Edge&& e, bool bidirectional, std::true_type)
  {
    view.delete_edge(e.descriptor());
    if (bidirectional && view.is_directed()) {
      view.delete_edge(reverse(e.descriptor()));
    }
  }

public:
  using result_type = void;

  //////////////////////////////////////////////////////////////////////
  /// @param p The probability of rewiring each edge.
  /// @param n  Size of the graph.
  /// @param bidirectional True to add backedges, false otherwise.
  /// @param max_proc Maximum number of locations used in a scaling study
  /// @param seed The seed for random-number generation.
  //////////////////////////////////////////////////////////////////////
  ws_rewire(double p, size_t n, bool bidirectional, size_t max_proc,
            unsigned int seed = get_location_id())
    : rand_gen(seed), m_p(p), m_n(n), m_bidirectional(bidirectional),
      m_max_proc(max_proc), m_vert_per_proc(n/max_proc),
      m_nlocs(get_num_locations()), m_loc_id(get_location_id()),
      m_start_proc(max_proc*m_loc_id/m_nlocs)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    size_t desc = v.descriptor();
    if (desc % m_vert_per_proc == 0) {
      size_t logical_proc = (desc/m_vert_per_proc)%(m_max_proc/m_nlocs);
      srand48((m_start_proc+logical_proc+65)*4321);
    }

    for (auto const& e : v) {
      if (this->rand(100) < m_p*100) {
        size_t t;

        do {
          t = this->rand(m_n);
        } while (t == desc);

        this->delete_edge(view, e, m_bidirectional,
          std::integral_constant<bool, Delete>{}
        );

        view.add_edge_async(desc, t);
        if (m_bidirectional && view.is_directed()) {
          view.add_edge_async(t, desc);
        }
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_p);
    t.member(m_n);
    t.member(m_bidirectional);
    t.member(m_max_proc);
    t.member(m_vert_per_proc);
    t.member(m_nlocs);
    t.member(m_loc_id);
    t.member(m_start_proc);
  }
};

}

//////////////////////////////////////////////////////////////////////
/// @brief Generate a Watts-Strogatz small-world graph [^1].
///
/// First create a ring over n nodes.  Then each node in the ring is
/// connected with its k nearest neighbors (k-1 neighbors if k is odd).
/// Then shortcuts are created by replacing some edges as follows:
/// for each edge u-v in the underlying "n-ring with k nearest neighbors"
/// with probability p replace it with a new edge u-w with uniformly
/// random choice of existing node w.
///
/// In contrast with Newman-Watts-Strogatz, the random
/// rewiring does not increase the number of edges. The rewired graph
/// is not guaranteed to be connected.
///
/// @param g View of the graph to generate.
/// @param n The number of nodes in the graph.
/// @param k Each node is connected to k nearest neighbors in ring topology.
/// @param p The probability of rewiring each edge.
/// @param bidirectional True for adding back-edges, False for forward only.
/// @return The original view, now containing the generated graph.
///
/// [^1] Duncan J. Watts and Steven H. Strogatz, Collective dynamics of
///      small-world networks, Nature, 393, pp. 440--442, 1998.
//////////////////////////////////////////////////////////////////////

template <typename GraphView>
GraphView make_watts_strogatz(GraphView& g, size_t n, size_t k, double p,
                              bool bidirectional=true,
                              size_t max_procs=get_num_locations())
{
  stapl_assert(p <= 1.0 && p >= 0.0,
               "Watts-Strogatz probability must be between 0 and 1");
  stapl_assert(k <= n, "Watts-Strogatz k must less than n");

  typedef typename detail::ws_neighbors ef1_t;
  for (size_t i = 1; i < k/2 + 1; ++i) {
    ef1_t ef1(i, n, bidirectional);
    make_generator<GraphView, ef1_t>(g, n, ef1)();
  }

  typedef typename detail::ws_rewire<true> ef2_t;
  ef2_t ef2(p, n, bidirectional, max_procs);
  make_generator<GraphView, ef2_t>(g, n, ef2)();
  return g;
}

//////////////////////////////////////////////////////////////////////
/// @brief Generate a Watts-Strogatz small-world graph [^1].
///
/// First create a ring over n nodes.  Then each node in the ring is
/// connected with its k nearest neighbors (k-1 neighbors if k is odd).
/// Then shortcuts are created by replacing some edges as follows:
/// for each edge u-v in the underlying "n-ring with k nearest neighbors"
/// with probability p replace it with a new edge u-w with uniformly
/// random choice of existing node w.
///
/// In contrast with Newman-Watts-Strogatz, the random
/// rewiring does not increase the number of edges. The rewired graph
/// is not guaranteed to be connected.
///
/// The returned view owns its underlying container.
///
/// @param n The number of nodes in the graph
/// @param k Each node is connected to k nearest neighbors in ring topology
/// @param p The probability of rewiring each edge
/// @param bidirectional True for adding back-edges, False for forward only.
/// @return A view over the generated graph.
///
/// @b Example
/// @snippet watts_strogatz.cc Example
///
/// [^1] Duncan J. Watts and Steven H. Strogatz, Collective dynamics of
///      small-world networks, Nature, 393, pp. 440--442, 1998.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_watts_strogatz(size_t n, size_t k, double p,
                              bool bidirectional=true,
                              size_t max_procs=get_num_locations())
{
  stapl_assert(p <= 1.0 && p >= 0.0,
               "Watts-Strogatz probability must be between 0 and 1");
  stapl_assert(k <= n, "Watts-Strogatz k must less than n");

  GraphView g;

  typedef typename detail::ws_neighbors ef1_t;
  g = make_generator<GraphView, ef1_t>(n, ef1_t(1, n, bidirectional))();
  for (size_t i = 2; i < k/2 + 1; ++i) {
    make_generator<GraphView, ef1_t>(g, n, ef1_t(i, n, bidirectional))();
  }

  size_t logical_proc = max_procs/stapl::get_num_locations();
  size_t m_start_proc = logical_proc*stapl::get_location_id();
  srand48((m_start_proc+65)*4321);
  typedef typename detail::ws_rewire<true> ef2_t;
  ef2_t ef2(p, n, bidirectional, max_procs);
  make_generator<GraphView, ef2_t>(g, n, ef2)();
  return g;
}


//////////////////////////////////////////////////////////////////////
/// @brief Generate a Newman-Watts-Strogatz small-world graph [^1].
///
/// First create a ring over n nodes.  Then each node in the ring is
/// connected with its k nearest neighbors (k-1 neighbors if k is odd).
/// Then shortcuts are created by adding new edges as follows:
/// for each edge u-v in the underlying "n-ring with k nearest neighbors"
/// with probability p add a new edge u-w with randomly-chosen existing
/// node w.  In contrast with Watts-Strogatz, no edges are removed.
///
/// This function mutates the input graph.
///
/// @param g View of the graph to generate.
/// @param n The number of nodes in the graph.
/// @param k Each node is connected to k nearest neighbors in ring topology.
/// @param p The probability of rewiring each edge.
/// @param bidirectional True for adding back-edges, False for forward only.
/// @return The original view, now containing the generated graph.
///
/// [^1] M. E. J. Newman and D. J. Watts, Renormalization group analysis of
///      the small-world network model, Physics Letters A, 263, 341, 1999.
///      http://dx.doi.org/10.1016/S0375-9601(99)00757-4
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_newman_watts_strogatz(GraphView& g, size_t n, size_t k, double p,
                                     bool bidirectional=true)
{
  stapl_assert(p <= 1.0 && p >= 0.0,
               "Watts-Strogatz probability must be between 0 and 1");
  stapl_assert(k <= n, "Watts-Strogatz k must less than n");

  typedef typename detail::ws_neighbors ef1_t;
  for (size_t i = 1; i < k/2 + 1; ++i) {
    make_generator<GraphView, ef1_t>(g, n, ef1_t(i, n, bidirectional))();
  }

  typedef typename detail::ws_rewire<false> ef2_t;
  ef2_t ef2(p, n, bidirectional, g.get_num_locations());
  make_generator<GraphView, ef2_t>(g, n, ef2)();
  return g;
}

//////////////////////////////////////////////////////////////////////
/// @brief Generate a Newman-Watts-Strogatz small-world graph [^1].
///
/// First create a ring over n nodes.  Then each node in the ring is
/// connected with its k nearest neighbors (k-1 neighbors if k is odd).
/// Then shortcuts are created by adding new edges as follows:
/// for each edge u-v in the underlying "n-ring with k nearest neighbors"
/// with probability p add a new edge u-w with randomly-chosen existing
/// node w.  In contrast with Watts-Strogatz, no edges are removed.
///
/// The returned view owns its underlying container.
///
///
/// @param n The number of nodes in the graph
/// @param k Each node is connected to k nearest neighbors in ring topology
/// @param p The probability of rewiring each edge
/// @param bidirectional True for adding back-edges, False for forward only.
/// @return A view over the generated graph.
/// @see make_watts_strogatz
///
/// @b Example
/// @snippet newman_watts_strogatz.cc Example
///
/// [^1] M. E. J. Newman and D. J. Watts, Renormalization group analysis of
///      the small-world network model, Physics Letters A, 263, 341, 1999.
///      http://dx.doi.org/10.1016/S0375-9601(99)00757-4
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_newman_watts_strogatz(size_t n, size_t k, double p,
                                     bool bidirectional=true)
{
  stapl_assert(p <= 1.0 && p >= 0.0,
               "Watts-Strogatz probability must be between 0 and 1");
  stapl_assert(k <= n, "Watts-Strogatz k must less than n");

  GraphView g;

  typedef typename detail::ws_neighbors ef1_t;
  g = make_generator<GraphView, ef1_t>(n, ef1_t(1, n, bidirectional))();
  for (size_t i = 2; i < k/2 + 1; ++i) {
    make_generator<GraphView, ef1_t>(g, n, ef1_t(i, n, bidirectional))();
  }

  typedef typename detail::ws_rewire<false> ef2_t;
  make_generator<GraphView, ef2_t>(g, n, ef2_t(p, n, bidirectional,
                                               g.get_num_locations()))();
  return g;
}

} // namespace generators

} // namespace stapl

#endif

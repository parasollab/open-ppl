/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_LOLLIPOP_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_LOLLIPOP_HPP

#include <stapl/containers/graph/generators/generator.hpp>
#include <stapl/containers/graph/generators/complete.hpp>
#include <stapl/containers/graph/generators/list.hpp>

namespace stapl {

namespace generators {

//////////////////////////////////////////////////////////////////////
/// @brief Generate a lollipop graph.
///
/// This is the barbell graph without the right barbell.
///
/// For $$m > 1$$ and $$n \\geq 0$$, the complete graph $$K_m$$ is connected to
/// the path $$P_n$$.  The resulting $$m+n$$ nodes are labelled
/// $$0, \\dots, m-1$$ for the complete graph and $$m, \\dots, m+n-1$$ for the
/// path. The 2 subgraphs are joined via the edge $$(m-1,m)$$.  If $$n=0$$,
/// this is merely a complete graph.
///
/// Node labels are the integers 0 to m+n-1.
///
/// This function mutates the input graph.
///
/// @note This graph is an extremal example in David Aldous and Jim
/// Fill's etext on Random Walks on Graphs. [^1]
///
/// @param g A view over the graph to generate.
/// @param m The number of nodes in the complete subgraph.
/// @param n The number of nodes in the chain.
/// @param bidirectional True for adding back-edges, False for forward only.
/// @return The original view, now containing the generated graph.
///
/// [^1] Aldous, David and Fill, James Allen. "Reversible Markov Chains and
///      Random Walks on Graphs," 2002.
///      https://www.stat.berkeley.edu/~aldous/RWG/book.html
/////////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_lollipop(GraphView& g, size_t m, size_t n,
                        bool bidirectional=true)
{
  typedef typename detail::complete_neighbors ef1_t;
  typedef typename detail::list_neighbors     ef2_t;

  // create complete subgraph
  if (m > 0)
    g = make_generator<GraphView, ef1_t>(g, m+n, ef1_t(m, bidirectional))();

  // create list subgraph
  if (n > 0)
    g = make_generator<GraphView, ef2_t>(g, m+n, ef2_t(n, m, bidirectional))();

  // connect the two
  if (get_location_id() == 0)
    if (n > 0 && m > 0) {
      g.add_edge_async(m-1, m);
      if (g.is_directed() && bidirectional)
        g.add_edge_async(m, m-1);
    }

  return g;
}


//////////////////////////////////////////////////////////////////////
/// @brief Generate a lollipop graph.
///
/// This is the barbell graph without the right barbell.
///
/// For $$m > 1$$ and $$n \\geq 0$$, the complete graph $$K_m$$ is connected to
/// the path $$P_n$$.  The resulting $$m+n$$ nodes are labelled
/// $$0, \\dots, m-1$$ for the complete graph and $$m, \\dots, m+n-1$$ for the
/// path. The 2 subgraphs are joined via the edge $$(m-1,m)$$.  If $$n=0$$,
/// this is merely a complete graph.
///
/// Node labels are the integers 0 to m+n-1.
///
/// This function mutates the input graph.
///
/// @note This graph is an extremal example in David Aldous and Jim
/// Fill's etext on Random Walks on Graphs. [^1]
///
/// @param m The number of nodes in the complete subgraph.
/// @param n The number of nodes in the chain.
/// @param bidirectional True for adding back-edges, False for forward only.
/// @return A view over the generated graph.
///
/// The returned view owns its underlying container.
///
/// @b Example
/// @snippet lollipop.cc Example
///
/// [^1] Aldous, David and Fill, James Allen. "Reversible Markov Chains and
///      Random Walks on Graphs," 2002.
///      https://www.stat.berkeley.edu/~aldous/RWG/book.html
/////////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_lollipop(size_t m, size_t n, bool bidirectional=true)
{
  typedef typename detail::complete_neighbors ef1_t;
  typedef typename detail::list_neighbors     ef2_t;

  GraphView g;

  // create complete subgraph
  g = make_generator<GraphView, ef1_t>(m+n, ef1_t(m, bidirectional))();

  // create list subgraph
  g = make_generator<GraphView, ef2_t>(g, m+n, ef2_t(n, m, bidirectional))();
  // connect the two
  if (get_location_id() == 0)
    if (n > 0 && m > 0) {
      g.add_edge_async(m-1, m);
      if (g.is_directed() && bidirectional)
        g.add_edge_async(m, m-1);
    }

  return g;
}


} // namespace generators

} // namespace stapl

#endif

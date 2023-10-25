/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PAGE_RANK_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PAGE_RANK_HPP

#include <stapl/containers/graph/algorithms/execute.hpp>

namespace stapl {

namespace page_rank_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Work-function to initialize the auxiliary and actual
/// PageRank values.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Rank>
struct page_rank_init_wf
{
  Rank m_initial_rank;
  Rank m_initial_aux_rank;
  typedef void result_type;

  page_rank_init_wf(Rank const& initial_rank, Rank const& initial_aux_rank)
    : m_initial_rank(initial_rank), m_initial_aux_rank(initial_aux_rank)
  { }

  template <typename T>
  void operator()(T v) const
  {
    v.property().rank(m_initial_rank);
    v.property().new_rank(m_initial_aux_rank);
  }

  void define_type(typer& t)
  {
    t.member(m_initial_rank);
    t.member(m_initial_aux_rank);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reducer functor for @ref page_rank().
///
/// Reduces two page_rank properties to update the first one.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct vp_reducer
{
  template<typename VP1, typename VP2>
  void operator()(VP1& p1, VP2& p2) const
  {
    p1.new_rank(p2.new_rank()+p1.new_rank());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to copy the auxiliary rank to the actual
/// PageRank values and reinitialize the auxiliary rank.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Rank>
struct page_rank_copy_wf
{
  Rank m_damping;
  size_t m_max_visits;
  typedef void result_type;

  page_rank_copy_wf(Rank const& damping, size_t max_visits)
    : m_damping(damping), m_max_visits(max_visits)
  { }

  template <typename Vertex>
  void operator()(Vertex v) const
  {
    v.property().rank(v.property().new_rank());
    v.property().new_rank(1 - m_damping);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Algorithm to copy the auxiliary rank to the actual
  /// PageRank values and reinitialize the auxiliary rank.
  ///
  /// Executed at the end of each paradigm iteration as a post-execute.
  /// @param g The input graph_view
  /// @param iteration The iteration value.
  //////////////////////////////////////////////////////////////////////
  template <typename GraphView>
  void operator()(GraphView& g, size_t iteration) const
  {
    if (iteration < m_max_visits)
      map_func(*this, g);
  }

  void define_type(typer& t)
  {
    t.member(m_damping);
    t.member(m_max_visits);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to add the value of the incomming PageRank to the
/// target vertex's auxiliary PageRank.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Rank>
struct update_func
{
  Rank m_rank;

  typedef bool result_type;

  update_func(Rank const& rank = 0.0)
    : m_rank(rank)
  { }

  template <class Vertex>
  bool operator()(Vertex target) const
  {
    target.property().new_rank(m_rank + target.property().new_rank());
    return true;
  }

  void define_type(typer& t)
  { t.member(m_rank); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to compute the PageRank of a vertex and push it
/// to the vertex's neighbors.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Rank>
struct page_rank_wf
{
  Rank m_damping;

  using concurrency_model = sgl::weak_concurrency;

  page_rank_wf(Rank const& damping)
    : m_damping(damping)
  { }

  template<typename Vertex, typename GraphVisitor>
  bool operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    Rank degree = graph_visitor.degree(v);
    Rank u_rank_out = m_damping*v.property().rank()/degree;
    graph_visitor.visit_all_edges(std::forward<Vertex>(v),
                                  update_func<Rank>(u_rank_out));
    return false;
  }

  void define_type(typer& t)
  { t.member(m_damping); }
};

}; // namespace page_rank_impl;


//////////////////////////////////////////////////////////////////////
/// @brief Parallel Level-Synchronized PageRank Algorithm.
///
/// Performs a PageRank on the input @ref graph_view, storing the ranks
/// of all vertices on their properties.
/// @param policy A policy for execution.
/// @param graph The @ref graph_view over the input graph.
/// @param iterations The number of PageRank iterations to perform.
/// @param damping The damping factor for the algorithm.
/// @return The number of iterations performed.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Policy, typename GView>
size_t page_rank(Policy&& policy, GView& graph, size_t iterations,
                 double damp = 0.85)
{
  using namespace page_rank_impl;

  using rank_type = typename GView::vertex_property::value_type;
  using vertex_op = page_rank_wf<rank_type>;
  using neighbor_op = update_func<rank_type>;

  rank_type damping = static_cast<rank_type>(damp);

  // Initialize ranks.
  rank_type normalized_sz = 1;
  map_func(page_rank_init_wf<rank_type>(normalized_sz, 1-damping), graph);

  page_rank_copy_wf<rank_type> post{damping, iterations};

  for (std::size_t i = 0; i < iterations; ++i) {
    sgl::execute(
      std::forward<Policy>(policy), graph, vertex_op{damping}, neighbor_op{},
      vp_reducer{});
    post(graph, i);
  }

  return iterations;
}

//////////////////////////////////////////////////////////////////////
/// @brief Parallel Level-Synchronized PageRank Algorithm.
///
/// Performs a PageRank on the input @ref graph_view, storing the ranks
/// of all vertices on their properties.
/// @param graph The @ref graph_view over the input graph.
/// @param iterations The number of PageRank iterations to perform.
/// @param damping The damping factor for the algorithm.
/// @return The number of iterations performed.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GView>
size_t page_rank(GView& graph, size_t iterations, double damp = 0.85)
{
  return page_rank(stapl::sgl::make_execution_policy("lsync", graph), graph,
                   iterations, damp);
}

} // namespace stapl
#endif

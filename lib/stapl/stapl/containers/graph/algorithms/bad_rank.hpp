/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_BAD_RANK_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_BAD_RANK_HPP

#include <vector>
#include <utility>
#include <stapl/views/repeated_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/algorithms/execute.hpp>

namespace stapl {

namespace bad_rank_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Work-function to copy the blacklisted status of each vertex
/// of one graph onto another
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct copy_bl_status_wf
{
  typedef void result_type;

  template <typename V1, typename V2>
  result_type operator()(V1 original, V2 duplicate)
  {
    duplicate.property().set_blacklisted(
        original.property().is_blacklisted());
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work-function to create reversed copies of each edge within
/// a second graph
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct copy_reversed_edges_wf
{
  typedef void result_type;

  template <typename Vertex, typename GraphView>
  result_type operator()(Vertex v, GraphView edge_reverse_view)
  {
    typename Vertex::adj_edge_iterator it = v.begin(), it_e = v.end();
    for (; it != it_e; ++it)
      edge_reverse_view.add_edge((*it).target(), v.descriptor());
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work-function to copy the computed rank back to the
/// original graph
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct copy_rank_wf
{
  typedef void result_type;

  template <typename V1, typename V2>
  result_type operator()(V1 duplicate, V2 original)
  {
    original.property().rank(duplicate.property().rank());
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work-function to initialize the auxiliary and actual
/// BadRank values
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class bad_rank_init_wf
{
  double m_damping;
  size_t m_num_blacklisted;

public:
  bad_rank_init_wf(double const& damping, size_t num_bl)
    : m_damping(damping), m_num_blacklisted(num_bl)
  { }

  typedef void result_type;
  template <typename T>
  result_type operator()(T v) const
  {
    double init_rank = (v.property().is_blacklisted() * m_damping)
      / m_num_blacklisted;

    v.property().rank(init_rank);
    v.property().new_rank(init_rank);
    v.property().set_active(true);
  }

  void define_type(typer& t)
  {
    t.member(m_damping);
    t.member(m_num_blacklisted);
  }
};

///////////////////////////////////////////////////////////////////////
/// @brief Update function to add the value of the incoming BadRank to
/// to the target vertex's auxilary BadRank
/// @ingroup pgraphAlgoDetails
///////////////////////////////////////////////////////////////////////
class rank_update_func
{
  double m_rank;

public:
  rank_update_func(double const& rank = 0.0)
    : m_rank(rank)
  { }

  typedef bool result_type;
  template <class Vertex>
  result_type operator()(Vertex&& tgt) const
  {
    tgt.property().new_rank(m_rank + tgt.property().new_rank());
    return true;
  }

  void define_type(typer& t)
  { t.member(m_rank); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work-function to copy the auxiliary rank to the actual
/// BadRank values and reinitialize the auxiliary rank.
/// Executed at the end of each paradigm iteration as a post-execute.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class bad_rank_copy_wf
{
  double m_damping;
  size_t m_num_blacklisted;
  size_t m_max_visits;

public:
  bad_rank_copy_wf(double const& damping, size_t num_bl, size_t max_visits)
    : m_damping(damping), m_num_blacklisted(num_bl), m_max_visits(max_visits)
  { }

  typedef void result_type;
  template <typename Vertex>
  void operator()(Vertex v) const
  {
    v.property().rank(v.property().new_rank());
    v.property().new_rank(v.property().is_blacklisted() * m_damping
                          / m_num_blacklisted);
    v.property().set_active(true);
  }

  //////////////////////////////////////////////////////////////////////
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
    t.member(m_num_blacklisted);
    t.member(m_max_visits);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work-function to compute the BadRank of a vertex from the
/// vertex's neighbors.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class bad_rank_wf
{
  double m_damping;

public:
  bad_rank_wf(double const& damping)
    : m_damping(damping)
  { }

  typedef bool result_type;
  template<typename Vertex, typename GraphVisitor>
  bool operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    if (v.property().is_active())
    {
      double u_rank_pulled = m_damping * v.property().rank()
        / double(graph_visitor.degree(v));

      graph_visitor.visit_all_edges(std::forward<Vertex>(v),
                                    rank_update_func(u_rank_pulled));

      v.property().set_active(false);
      return true;
    }
    return false;
  }

  void define_type(typer& t)
  {
    t.member(m_damping);
  }
};

//////////////////////////////////////////////////////////////////////
/// @bried Reducer functor for @ref bad_rank()
///
/// Reduces two bad_rank properties to update the first one.
//////////////////////////////////////////////////////////////////////
struct vp_reducer
{
  template<typename VP1, typename VP2>
  void operator()(VP1&& p1, VP2&& p2) const
  {
    p1.new_rank(p2.new_rank()+ p1.new_rank());
  }
};

};  // namespace bad_rank_impl;

//////////////////////////////////////////////////////////////////////
/// @brief Parallel BadRank Algorithm.
///
/// Computes the BadRank values of the vertices of the input @ref graph_view,
/// storing the ranks of all vertices on their properties.
/// @param policy A policy for execution.
/// @param graph The @ref graph_view over the input graph.
/// @param iterations The number of BadRank iterations to perform.
/// @param num_bl The number of blacklisted vertices in the graph.
/// @param damping The damping factor for the algorithm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Policy, typename GView>
size_t bad_rank(Policy&& policy, GView& graph, size_t iterations, size_t num_bl,
  double damping = 0.85)
{
  typedef stapl::multidigraph<stapl::properties::bad_rank_property> PGR_static;
  typedef graph_view<PGR_static> graph_view_t;

  using namespace bad_rank_impl;

  PGR_static p_static(graph.size());
  graph_view_t reversed_graph(p_static);

  // Copy the blacklisted status of each vertex to the reversed graph
  map_func(copy_bl_status_wf(), graph, reversed_graph);
  // Add a reversed copy of each edge to the reversed graph
  map_func(copy_reversed_edges_wf(), graph,
      stapl::make_repeat_view(reversed_graph));
  // Initialize ranks
  map_func(bad_rank_impl::bad_rank_init_wf(damping, num_bl), reversed_graph);

  bad_rank_copy_wf post{damping, num_bl, iterations};

  for (std::size_t i = 0; i < iterations; ++i)  {
    sgl::execute(
        std::forward<Policy>(policy), graph, bad_rank_wf{damping},
        rank_update_func{}, vp_reducer{});
    //copy rank to the original graph
    post(graph, i);
  }

  return iterations;
}

} // namespace stapl
#endif

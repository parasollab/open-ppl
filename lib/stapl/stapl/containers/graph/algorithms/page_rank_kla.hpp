/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PAGE_RANK_KLA_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PAGE_RANK_KLA_HPP

#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/aggregator.hpp>
#include <stapl/containers/graph/algorithms/paradigms/graph_paradigm.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/numeric.hpp>

namespace stapl {

namespace page_rank_kla_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Initialize the KLA PageRank property
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class initialize_property
{
  double m_initial_rank, m_initial_new_rank;
  size_t m_iter;

public:
  using result_type = void;

  initialize_property(double d1, double d2, size_t num_iter)
    : m_initial_rank(d1), m_initial_new_rank(d2), m_iter(num_iter)
  { }

  template<typename V>
  result_type operator()(V v)
  {
    v.property().in_degree(0);
    v.property().iteration(m_iter);
    v.property().update_last_sent_iteration(0);
    v.property().initialize_rank(m_initial_rank);
  }

  void define_type(typer& t)
  {
    t.member(m_iter);
    t.member(m_initial_rank);
    t.member(m_initial_new_rank);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Neighbor operator used to compute the in-degree for each vertex
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct degree_neighbor_operator
{
  using result_type = bool;

  template<typename Neighbor>
  result_type operator()(Neighbor u) const
  {
    u.property().in_degree(u.property().in_degree() + 1);
    return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex operator used to compute the in-degree for each vertex
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct degree_vertex_operator
{
  using result_type = bool;

  template<typename Vertex, typename Visitor>
  result_type operator()(Vertex v, Visitor visitor) const
  {
    visitor.visit_all_edges(std::forward<Vertex>(v),
                            degree_neighbor_operator());
    return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Neighbor operator for the KLA PageRank algorithm
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class neighbor_operator
{
  double m_rank;
  size_t m_iter;

public:
  using result_type = bool;

  neighbor_operator(double rank = 0., size_t iter = 0)
    : m_rank(rank), m_iter(iter)
  { }

  template <class Vertex>
  result_type operator()(Vertex target) const
  {
    stapl_assert(target.property().in_degree() > 0,
                 "This vertex should never receive a visit");
    stapl_assert(target.property().last_sent_iteration() < m_iter,
                 "Received update from past iteration");

    return target.property().future_add(m_iter, m_rank);
  }

  void define_type(typer& t)
  {
    t.member(m_rank);
    t.member(m_iter);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex operator for the KLA PageRank algorithm
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class vertex_operator
{
public:
  typedef bool result_type;

  template<class Vertex, class GraphVisitor>
  result_type operator()(Vertex v, GraphVisitor graph_visitor) const
  {
    stapl_assert(v.property().in_degree() > 0,
                 "Algorithm not defined for dangling pages");

    bool was_active = false;

    const std::size_t last_sent_iteration = v.property().last_sent_iteration();
    const std::size_t highest_available_iteration = v.property().iteration();

    for (std::size_t i = last_sent_iteration;
         i <= highest_available_iteration && i < v.property().max_iteration();
         ++i)
    {
      // Only send iterations that have values to send
      if (!v.property().is_ready(i) || v.property().was_sent(i)) {
        continue;
      }

      double u_rank_out = 0.85 * v.property().rank(i) / double(v.size());
      neighbor_operator neighbor_op{ u_rank_out, i + 1 };

      // Send my rank for the next iteration
      v.property().mark_as_sent(i);
      graph_visitor.visit_all_edges(v, neighbor_op);

      was_active = true;
    }

    // Vertex was not active
    return was_active;
  }
};

} //namespace page_rank_kla_detail


//////////////////////////////////////////////////////////////////////
/// @brief Parallel k-level-asynchronous PageRank Algorithm.
///
/// Performs a PageRank on the input @ref graph_view, storing the ranks
/// of all vertices on their properties.
//
/// @param graph The @ref graph_view over the input graph.
/// @param num_iter The number of PageRank iterations to perform.
/// @param k The level of asynchrony.
/// @return The number of iterations performed.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GView>
size_t page_rank_kla(GView& g, size_t num_iter, size_t k)
{
  using namespace page_rank_kla_detail;

  using vertex_op_t = vertex_operator;
  using neighbor_op_t = neighbor_operator;

  // Initialize the property
  map_func(initialize_property(1, 0.15, num_iter), g);

  // Compute the in-degree for each vertex
  graph_paradigm(degree_vertex_operator(), degree_neighbor_operator(), g, 0);

  return graph_paradigm(vertex_op_t(), neighbor_op_t(), g, k);
}

} // namespace stapl

#endif

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_APPROXIMATE_BREADTH_FIRST_SEARCH_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_APPROXIMATE_BREADTH_FIRST_SEARCH_HPP

#include <stapl/containers/graph/algorithms/paradigms/kla_paradigm.hpp>
#include <stapl/containers/graph/generators/generator_base.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Initialize the properties for a vertex
//////////////////////////////////////////////////////////////////////
template <class VD, class LevelType>
class approx_bfs_init_wf
{
  typedef VD vd_type;
  vd_type    m_source;

public:
  typedef void result_type;

  approx_bfs_init_wf(vd_type source)
    : m_source(source)
  { }

  template <class Vertex>
  void operator()(Vertex v)
  {
    if (v.descriptor() == m_source) {
      v.property().level(LevelType(1));
      v.property().active(true);
    } else {
      v.property().level(LevelType(0));
      v.property().active(false);
    }
    v.property().parent(v.descriptor());
  }

  void define_type(typer& t)
  { t.member(m_source); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Neighbor operator for @ref approximate_breadth_first_search().
///
/// Updates the target vertex with BFS-parent and BFS-level
/// information if the target vertex has not been visited before, or
/// if the target's level is greater than the incoming level.
/// Repropagates this new distance if it is sufficiently better than the
/// previously propagated value. That is, if the relative error between
/// this new distance and the previously propagated value is greater than
/// tau, then this vertex will become active.
/// @tparam VD Type of the vertex-descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VD>
class approx_bfs_neighbor_op
{
  using parent_type = VD;
  static constexpr double epsilon = 0.00001;

public:
  parent_type m_parent;
  size_t      m_level;
  double      m_tau;

  using result_type = bool;

  //////////////////////////////////////////////////////////////////////
  /// @param p The BFS-parent of the target vertex.
  /// @param level The BFS-level of the target vertex.
  /// @param tau Approximation factor
  //////////////////////////////////////////////////////////////////////
  approx_bfs_neighbor_op(parent_type p = 0, size_t level = 0, double tau = 0.0)
    : m_parent(p), m_level(level), m_tau(tau)
  { }

  template <class Vertex>
  result_type operator()(Vertex&& target) const
  {
    const bool first_visit = target.property().level() == 0;
    const bool better_level =
      first_visit || target.property().level() > m_level;

    if (better_level)
    {
      // Always update level if it's better
      target.property().level(m_level);
      target.property().parent(m_parent);

      const double relative_difference =
        (double(target.property().propagated_level()) - double(m_level))
          / double(m_level);

      const bool meets_threshold =
        relative_difference > m_tau ||
        std::fabs(relative_difference - m_tau) < epsilon;

      if (first_visit || meets_threshold) {
        target.property().active(true);
        target.property().propagated_level(m_level);
        return true;
      }
    }

    return false;
  }

  void define_type(typer& t)
  {
    t.member(m_parent);
    t.member(m_level);
    t.member(m_tau);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex operator for @ref approximate_breadth_first_search().
///
/// A vertex is visited if it is active. Active vertices update their
/// neighbors with new BFS level and parent information.
/// Returns true if vertex was active, false otherwise.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class approx_bfs_vertex_op
{
  double m_tau;

public:
  approx_bfs_vertex_op(double tau)
    : m_tau(tau)
  { }

  using result_type = bool;

  template<class Vertex, class GraphVisitor>
  result_type operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    using descriptor_type =
      typename std::decay<Vertex>::type::vertex_descriptor;

    if (v.property().active())
    {
      v.property().active(false);

      graph_visitor.visit_all_edges(std::forward<Vertex>(v),
        approx_bfs_neighbor_op<descriptor_type>(
          v.descriptor(), v.property().level()+1, m_tau
        )
      );
      return true;
    }

    return false;
  }

  void define_type(typer& t)
  {
    t.member(m_tau);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Builds an approximate breadth-first search tree for a graph.
///
/// Performs a breadth-first search on the input @ref graph_view, storing
/// the BFS-level and BFS-parent on each reachable vertex. The BFS-level
/// stored is at most k times the true BFS-level.
/// @param g The @ref graph_view over the input graph.
/// @param source The descriptor of the source vertex for this traversal.
/// @param k The maximum amount of asynchrony allowed in each phase.
/// @param tau A parameter that controls the amount of error allowed upon
///            a visitation.
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<class GView>
size_t approximate_breadth_first_search(GView& g,
                            typename GView::vertex_descriptor const& source,
                            size_t k = 0, double tau = 1.0)
{
  using vd_type = typename GView::vertex_descriptor;
  using neighbor_op_t = approx_bfs_neighbor_op<vd_type>;
  using vertex_op_t = approx_bfs_vertex_op;

  vertex_op_t vop(tau);
  neighbor_op_t nop;

  // Initialize the vertices.
  map_func(approx_bfs_init_wf<vd_type, size_t>(source), g);

  return kla_paradigm(vop, nop, g, k);
}

} // namespace stapl

#endif

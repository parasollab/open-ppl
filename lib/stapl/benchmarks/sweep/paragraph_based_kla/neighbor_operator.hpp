/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_KLA_NEIGHBOR_OPERATOR_PG_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_KLA_NEIGHBOR_OPERATOR_PG_HPP

#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/paragraph/paragraph.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/localize_element.hpp>

namespace stapl {

namespace kla_pg {

namespace kla_pg_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to visit the target vertex with the user provided
/// neighbor-operator.
///
/// Implements the KLA visit pattern, which invokes the user's
/// neighbor-operator on the visited vertex and reinvokes the
/// vertex operator if the both the visit was successful and
/// the sufficient propagation condition is met (e.g., the
/// current level is less than the max allowed level for the
/// KLA superstep).
///
/// @tparam VertexOp Type of the user provided vertex-operator.
/// @tparam NeighborOp Type of the user provided neighbor-operator.
/// @tparam Derived The derived neighbor operator wrapper which
/// implements the logic for propagation
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename VertexOp, typename NeighborOp, typename Derived>
struct neighbor_operator_apply_base
{
  VertexOp   m_wf;
  NeighborOp m_uf;

  Derived const& derived() const
  {
    return static_cast<Derived const&>(*this);
  }

public:
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param wf A copy of the currently executing vertex_operator_apply,
  /// containing information about the current and the max KLA-level of
  /// this visit.
  /// @param uf The user provided neighbor-operator.
  //////////////////////////////////////////////////////////////////////
  neighbor_operator_apply_base(VertexOp const& wf, NeighborOp const& uf)
    : m_wf(wf), m_uf(uf)
  {
    // Increment the level by one.
    m_wf.increment_level();
  }

  template<typename PGV, typename Vertex, typename GraphView>
  void operator()(PGV& pgv, Vertex&& target, GraphView& gvw)
  {
    // invoke the neighbor operator and record whether or not we should
    // reinvoke the vertex operator on this vertex afterwards
    const bool reinvoke = m_uf(std::forward<Vertex>(target));

    // reinvoke the vertex operator if the sufficient propagation
    // condition holds (e.g., we have not reached the end of a
    // KLA superstep)
    if (reinvoke && derived().propagation_condition(target))
      m_wf(pgv, std::forward<Vertex>(target), gvw);
  }

  template<typename PGV, typename Vertex>
  void operator()(PGV& pgv, Vertex&& target)
  {
    // invoke the neighbor operator and record whether or not we should
    // reinvoke the vertex operator on this vertex afterwards
    const bool reinvoke = m_uf(std::forward<Vertex>(target));
    // reinvoke the vertex operator if the sufficient propagation
    // condition holds (e.g., we have not reached the end of a
    // KLA superstep)
    if (reinvoke && derived().propagation_condition(target)) {
      m_wf(pgv, std::forward<Vertex>(target));
    }
  }

  VertexOp const& vertex_op() const
  {
    return m_wf;
  }

  NeighborOp const& neighbor_op() const
  {
    return m_uf;
  }

  void define_type(typer& t)
  {
    t.member(m_wf);
    t.member(m_uf);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to visit the target vertex with the user provided
/// neighbor-operator.
///
/// This is a facade for @see neighbor_operator_apply_base where
/// the trigger for propagation is only if the level criterion
/// is met (i.e., revisitation will be in the same KLA superstep).
///
/// @tparam AvoidHubs Flag to enable or disable hub optimization
/// @tparam VertexOp Type of the user provided vertex-operator.
/// @tparam NeighborOp Type of the user provided neighbor-operator.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<bool AvoidHubs, typename VertexOp, typename NeighborOp>
struct neighbor_operator_apply
 : public neighbor_operator_apply_base<VertexOp, NeighborOp,
     neighbor_operator_apply<AvoidHubs, VertexOp, NeighborOp>
   >
{
  typedef neighbor_operator_apply_base<VertexOp, NeighborOp,
     neighbor_operator_apply<AvoidHubs, VertexOp, NeighborOp>
   > base_type;

public:
  template<typename... Args>
  neighbor_operator_apply(Args&&... args)
   : base_type(std::forward<Args>(args)...)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Condition to determine if a vertex operator should be invoked
  ///        on the same vertex after the neighbor operator. In this case,
  ///        we only consider the KLA superstep condition.
  /// @param target The vertex that was sucessfully visited
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex>
  bool propagation_condition(Vertex&&) const
  {
    return this->vertex_op().level() <= this->vertex_op().max_level();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to visit the target vertex with the user provided
/// neighbor-operator.
///
/// This is a specialization for @see neighbor_operator_apply_base where
/// the trigger for propagation is both the level criterion
/// and that the visited vertex's out-degree falls below a threshold.
///
/// @tparam VertexOp Type of the user provided vertex-operator.
/// @tparam NeighborOp Type of the user provided neighbor-operator.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename VertexOp, typename NeighborOp>
struct neighbor_operator_apply<true, VertexOp, NeighborOp>
 : public neighbor_operator_apply_base<VertexOp, NeighborOp,
     neighbor_operator_apply<true, VertexOp, NeighborOp>
   >
{
  typedef neighbor_operator_apply_base<VertexOp, NeighborOp,
     neighbor_operator_apply<true, VertexOp, NeighborOp>
   > base_type;

public:
  template<typename... Args>
  neighbor_operator_apply(Args&&... args)
   : base_type(std::forward<Args>(args)...)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Condition to determine if a vertex operator should be invoked
  ///        on the same vertex after the neighbor operator. In this case,
  ///        we consider both the KLA superstep condition and if this
  ///        vertex's out-degree is below a given threshold (which is defined
  ///        in the vertex operator wrapper (@see vertex_operator_apply)).
  /// @param target The vertex that was sucessfully visited
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex>
  bool propagation_condition(Vertex&& target) const
  {
    return this->vertex_op().level() <= this->vertex_op().max_level() &&
     target.size() < this->vertex_op().degree_threshold();
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

} // namespace kla_pg_detail

} // namespace kla_pg

} // namespace stapl

#endif

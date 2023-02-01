/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_KLA_VERTEX_OPERATOR_PG_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_KLA_VERTEX_OPERATOR_PG_HPP

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/views/localize_element.hpp>
#include "neighbor_operator.hpp"

namespace stapl {

namespace kla_pg {

namespace kla_pg_detail {

struct false_predicate
{
  template<typename Vertex>
  bool operator()(Vertex&&)
  { return false;}
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function to wrap the user provided vertex-operator.
///
/// The work function is applied on a vertex if the vertex is active.
/// Active vertices may perform some computation and update their values, and
/// may visit their neighboring vertices with the user provided
/// neighbor-operator.
/// Returns true if vertex was active (i.e. the user's work function returned
/// true), false otherwise.
///
/// @tparam AvoidHubs Flag to enable the hub avoidance optimization. When a
/// vertex is visited with the neighbor operator, the vertex operator will
/// not be invoked on it if its degree exceeds a user-defined threshold.
/// @tparam GraphView Type of the input graph view.
/// @tparam VertexOp Type of the user provided work function.
/// @tparam Derived vertex_operator_apply class, which uses CRTP
/// to provide certain interfaces (such as hub avoidance)
/// @tparam Predicate Type of a predicate work function that recieves a vertex
/// and returns a bool indicating whether or not to terminate. This
/// defaults to a false_predicate which always returns false.
/// @ingroup pgraphAlgoDetails
/// @todo This work function stores a view to the input graph that is
/// needed for calling apply_set on target vertices inorder to apply
/// the neighbor-operator. This is currently passed in through the operator() as
/// a @ref repeat_view, with the pointer being set to the view received in
/// the operator(). However, this is slower than passing the view through
/// the constructor by ~7% on Hopper (using the g500 benchmark test). This
/// is documented in GForge to-do task #1207.
//////////////////////////////////////////////////////////////////////
template<bool AvoidHubs, class GraphView, class VertexOp, class Derived,
          class Predicate = kla_pg_detail::false_predicate>
class vertex_operator_apply_base
  : dynamic_wf
{
protected:
  VertexOp m_wf;
  size_t   m_curr_level;
  size_t   m_max_level;
  Predicate m_pred;

public:
  typedef typename GraphView::vertex_descriptor descriptor_t;
  typedef typename GraphView::vertex_descriptor vertex_descriptor;
  typedef typename GraphView::vertex_property   vertex_property;
  typedef typename GraphView::edge_property     edge_property;

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Visitor helper for KLA. Provides functionality to visit
  /// neighboring vertices and encapsulates the add_task call. Adds a
  /// task on the provided neighboring vertex with the given neighbor-operator.
  /// @tparam PGV Type of the Paragraph-view (needed for add_task()).
  /// @ingroup pgraphAlgoDetails
  //////////////////////////////////////////////////////////////////////
  template<class PGV>
  struct graph_visitor
  {
    PGV& m_pgv;
    GraphView& m_gvw;
    vertex_operator_apply_base const& m_parent;

    //////////////////////////////////////////////////////////////////////
    /// @param pgv The Paragraph view where tasks will be added.
    /// @param parent A pointer to the vertex_operator_apply instance.
    //////////////////////////////////////////////////////////////////////
    graph_visitor(PGV& pgv, GraphView& gvw,
                  vertex_operator_apply_base const& parent)
      : m_pgv(pgv), m_gvw(gvw), m_parent(parent)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @todo Investigate whether this creates an unnecessary copy
    //////////////////////////////////////////////////////////////////////
    template<typename NeighborOp>
    neighbor_operator_apply<AvoidHubs, Derived,
      typename std::decay<NeighborOp>::type
    >
    derived_visitor(NeighborOp&& uf) const
    {
      typedef neighbor_operator_apply<AvoidHubs, Derived,
        typename std::decay<NeighborOp>::type
      > apply_t;

      return apply_t(
        *static_cast<Derived const*>(&m_parent), std::forward<NeighborOp>(uf)
      );
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief Visits the target vertex and apply the provided update functor.
    ///
    /// Adds a task on the provided neighboring vertex with the given
    /// neighbor-operator.
    /// @param target The vertex descriptor of the target vertex to visit.
    /// @param uf The neighbor-operator to apply on the target vertex.
    /// @ingroup pgraphAlgoDetails
    //////////////////////////////////////////////////////////////////////
    template<class NeighborOp>
    void visit(descriptor_t const& target, NeighborOp&& uf) const
    {
      m_pgv.add_task(
        derived_visitor(std::forward<NeighborOp>(uf)),
        localize_ref(m_pgv),
        localize_ref(m_gvw, target),
        localize_ref(m_gvw)
      );
    }

    template<class NeighborOp>
    void visit(descriptor_t const& target, NeighborOp&& uf,
               default_info priority) const
    {
      m_pgv.add_task(
        priority,
        derived_visitor(std::forward<NeighborOp>(uf)),
        localize_ref(m_pgv),
        localize_ref(m_gvw, target),
        localize_ref(m_gvw)
      );
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief Visits all neighbors of the provided source vertex and applies
    /// the provided update functor on each of them.
    ///
    /// Adds a task on all neighboring vertices with the given
    /// neighbor-operator.
    /// @param source The source vertex whose neighbors are being visited.
    /// @param uf The neighbor-operator to apply on the target vertex.
    /// @ingroup pgraphAlgoDetails
    //////////////////////////////////////////////////////////////////////
    template<class Vertex, class NeighborOp>
    void visit_all_edges(Vertex source, NeighborOp&& uf) const
    {
      typedef typename Vertex::adj_edge_iterator iter_t;

      iter_t it = source.begin();
      iter_t it_e = source.end();

      auto visit_f = derived_visitor(std::forward<NeighborOp>(uf));

      for (; it != it_e; ++it) {
        m_pgv.add_task(
          visit_f,
          localize_ref(m_pgv),
          localize_ref(m_gvw, (*it).target()),
          localize_ref(m_gvw)
        );
      }
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief Visits all neighbors of the provided source vertex with a
    ///        neighbor operator if the edge satisfies a given predicate.
    ///
    /// @param source The source vertex whose neighbors are being visited.
    /// @param uf The neighbor-operator to apply on the target vertex.
    /// @param pred The unary predicate to determine whether an edge should
    ///        be visited
    /// @ingroup pgraphAlgoDetails
    //////////////////////////////////////////////////////////////////////
    template<class Vertex, class NeighborOp, typename VisitPredicate>
    void visit_all_edges_if(Vertex source, NeighborOp&& uf,
                            VisitPredicate&& pred) const
    {
      typedef typename Vertex::adj_edge_iterator iter_t;

      iter_t it = source.begin();
      iter_t it_e = source.end();

      auto visit_f = derived_visitor(std::forward<NeighborOp>(uf));

      for (; it != it_e; ++it) {
        if (std::forward<VisitPredicate>(pred)(*it))
          m_pgv.add_task(
            visit_f,
            localize_ref(m_pgv),
            localize_ref(m_gvw, (*it).target()),
            localize_ref(m_gvw)
          );
      }
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief Degree of vertex v.
    //////////////////////////////////////////////////////////////////////
    template<typename V>
    std::size_t degree(V&& v) const
    {
      return v.size();
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief Adds an edge between the two given vertices with given property.
    /// The edge is added asynchronously and the method returns immediately.
    ///
    /// Edge is not guaranteed to be added until after current superstep ends.
    /// The edge may be added during the current superstep.
    /// @param source Descriptor of the source vertex.
    /// @param target Descriptor of the target vertex.
    /// @param ep Property of the edge.
    //////////////////////////////////////////////////////////////////////
    void add_edge(descriptor_t const& source, descriptor_t const& target,
                  edge_property const& ep = edge_property()) const
    {
      m_gvw.add_edge_async(source, target);
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief Deletes the edge between the given source and target vertices.
    /// The edge is deleted asynchronously.
    ///
    /// The edge is not guaranteed to have been deleted until after current
    /// superstep ends, but may be deleted in the current superstep.
    /// @param source Descriptor of the source vertex.
    /// @param target Descriptor of the target vertex.
    //////////////////////////////////////////////////////////////////////
    void delete_edge(descriptor_t const& source,
                     descriptor_t const& target) const
    {
      m_gvw.delete_edge(source, target);
    }

    size_t const& level() const
    { return m_parent.level(); }

    size_t const& max_level() const
    { return m_parent.max_level(); }
  };

public:
  typedef std::pair<bool,bool> result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Vertex Operator Apply Base
  /// @param wf work function to apply to the vertices.
  /// @param curr_level The current KLA-level of this visit.
  /// @param max_level The maximum KLA-level of this visit.
  /// @param pred a predicate which recieves a vertex and is
  /// used to decide whether to terminate early
  //////////////////////////////////////////////////////////////////////
  vertex_operator_apply_base(VertexOp const& wf, size_t curr_level,
                             size_t max_level,
                             Predicate pred = Predicate())
    : m_wf(wf), m_curr_level(curr_level), m_max_level(max_level),
      m_pred(std::move(pred))
  { }

  template<class PGV, class Vertex, class View>
  result_type operator()(PGV& pgv, Vertex v, View& g)
  {
    typedef graph_visitor<PGV> visitor_t;
    bool first = m_wf.template operator()<Vertex, visitor_t const&>
      (v, visitor_t(pgv, g, *this));
    bool second = (m_pred(v));
    return std::make_pair(first,second);
  }

  void increment_level()
  { ++m_curr_level; }

  size_t const& level() const
  { return m_curr_level; }

  size_t const& max_level() const
  { return m_max_level; }

  void increment_iteration(size_t k)
  {
    m_curr_level += (k + 1);
    m_max_level = m_curr_level + k;
  }

  void define_type(typer& t)
  {
    t.member(m_wf);
    t.member(m_curr_level);
    t.member(m_max_level);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to wrap the user provided vertex-operator.
///
/// The work function is applied on a vertex if the vertex is active.
/// Active vertices may perform some computation and update their values, and
/// may visit their neighboring vertices with the user provided
/// neighbor-operator.
/// Returns true if vertex was active (i.e. the user's work function returned
/// true), false otherwise.
///
/// This is a facade for @see vertex_operator_apply_base that does not provide
/// the hub avoidance optimization.
///
/// @tparam AvoidHubs Flag to enable the hub avoidance optimization. When a
/// vertex is visited with the neighbor operator, the vertex operator will
/// not be invoked on it if its degree exceeds a user-defined threshold.
/// @tparam GraphView Type of the input graph view.
/// @tparam VertexOp Type of the user provided work function.
/// @tparam Predicate Predicate function which recieves a vertex and is used to
/// check whether to terminate.
//////////////////////////////////////////////////////////////////////
template<bool AvoidHubs, class GraphView, class VertexOp, class Predicate =
kla_pg_detail::false_predicate>
class vertex_operator_apply
  : public vertex_operator_apply_base<AvoidHubs, GraphView, VertexOp,
      vertex_operator_apply<AvoidHubs, GraphView, VertexOp, Predicate>,
      Predicate
    >
{
  typedef vertex_operator_apply_base<AvoidHubs, GraphView, VertexOp,
    vertex_operator_apply<AvoidHubs, GraphView, VertexOp, Predicate>, Predicate
  > base_type;

public:
  template<typename... Args>
  vertex_operator_apply(Args&&... args)
   : base_type(std::forward<Args>(args)...)
  { }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to wrap the user provided vertex-operator.
///
/// The work function is applied on a vertex if the vertex is active.
/// Active vertices may perform some computation and update their values, and
/// may visit their neighboring vertices with the user provided
/// neighbor-operator.
/// Returns true if vertex was active (i.e. the user's work function returned
/// true), false otherwise.
///
/// This is a specialization for @see vertex_operator_apply to provide
/// the hub avoidance optimization.
///
/// @tparam GraphView Type of the input graph view.
/// @tparam VertexOp Type of the user provided work function.
/// @tparam Predicate Predicate function which recieves a vertex and is used to
/// check whether to terminate.
//////////////////////////////////////////////////////////////////////
template<class GraphView, class VertexOp, class Predicate>
class vertex_operator_apply<true, GraphView, VertexOp, Predicate>
  : public vertex_operator_apply_base<true, GraphView, VertexOp,
      vertex_operator_apply<true, GraphView, VertexOp, Predicate>, Predicate
    >
{
  typedef vertex_operator_apply_base<true, GraphView, VertexOp,
    vertex_operator_apply<true, GraphView, VertexOp, Predicate>, Predicate
  > base_type;

  std::size_t m_degree_threshold;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the work function.
  /// @param wf User's vertex operator
  /// @param curr_level The current level of the algorithm
  /// @param max_level The maximum level that this KLA SS will reach
  /// @param degree_threshold The out-degree of a vertex to consider it a hub
  //////////////////////////////////////////////////////////////////////
  template<typename Op>
  vertex_operator_apply(Op&& wf, size_t curr_level, size_t max_level,
                        std::size_t degree_threshold,
                        Predicate pred=Predicate())
    : base_type(std::forward<Op>(wf), curr_level, max_level, pred),
      m_degree_threshold(degree_threshold)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the degree threshold to consider a vertex a hub for
  ///        hub optimization.
  //////////////////////////////////////////////////////////////////////
  size_t const& degree_threshold() const
  { return m_degree_threshold; }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_degree_threshold);
  }
};

} // namespace kla_pg_detail

} // namespace kla_pg

} // namespace stapl

#endif

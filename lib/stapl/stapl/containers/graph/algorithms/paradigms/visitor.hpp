/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_KLA_VISITOR_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_KLA_VISITOR_HPP

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/views/localize_element.hpp>
#include <stapl/containers/graph/algorithms/paradigms/neighbor_operator.hpp>
#include <stapl/containers/graph/algorithms/paradigms/concurrency_model.hpp>
#include <stapl/utility/for_each_range.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine if a vertex's edge list is stored
///        contiguously.
//////////////////////////////////////////////////////////////////////
template<typename Vertex>
struct has_contiguous_edges
{
  using type = typename
    std::is_same<
      typename Vertex::adj_edge_iterator,
      typename std::vector<
        typename std::iterator_traits<
          typename Vertex::adj_edge_iterator
        >::value_type
      >::iterator
    >::type;
};

namespace kla_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper class to transport visitations through the graph
///
/// @tparam Guarded Whether the visits should be guarded with an RMI lock
//////////////////////////////////////////////////////////////////////
template<bool Guarded>
struct visit_transporter
{
  template<typename Graph, typename Vertex, typename Op>
  static void single(Graph* const g, Vertex&& v, Op&& op)
  {
    g->distribution().unordered_apply_set(std::forward<Vertex>(v),
                                          std::forward<Op>(op));
  }

  template <typename Graph, typename Iterator, typename Op>
  static void
  bulk(Graph* const g, size_t loc, Iterator begin, Iterator end, Op&& op)
  {
    g->aggregate_apply_async(loc, make_range(begin, end), std::forward<Op>(op));
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper class to transport visitations through the graph.
///        Specialization when the visits are guarded with an RMI lock
//////////////////////////////////////////////////////////////////////
template<>
struct visit_transporter<true>
{
  template<typename Graph, typename Vertex, typename Op>
  static void single(Graph* const g, Vertex&& v, Op&& op)
  {
    g->distribution().guarded_apply_set(std::forward<Vertex>(v),
                                        std::forward<Op>(op));
  }

  template <typename Graph, typename Iterator, typename Op>
  static void
  bulk(Graph* const g, size_t loc, Iterator begin, Iterator end, Op&& op)
  {
    g->distribution().guarded_aggregate_apply_async(
      loc, make_range(begin, end), std::forward<Op>(op));
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Visitor object that is passed as the second paramter to the
///        vertex operator for KLA algorithms
///
/// @tparam Parent The @c vertex_operator_apply class that creates this visitor
//////////////////////////////////////////////////////////////////////
template<typename Parent>
class kla_visitor
{
  using graph_type = typename Parent::graph_type;
  using derived_type = typename Parent::derived_type;
  using descriptor_type = typename graph_type::vertex_descriptor;
  using edge_property = typename graph_type::edge_property;
  using transporter_type
    = visit_transporter<stapl::sgl::has_strong_concurrency_model<
      typename Parent::vertex_operator_type>::type::value>;

  Parent const& m_parent;

  graph_type* graph() const { return m_parent.graph(); }

public:
  //////////////////////////////////////////////////////////////////////
  /// @param parent A reference to vertex_operator_apply instance.
  //////////////////////////////////////////////////////////////////////
  kla_visitor(Parent const& parent)
    : m_parent(parent)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a wrapped neighbor operator that will be sent.
  //////////////////////////////////////////////////////////////////////
  template<typename NeighborOp>
  typename Parent::template make_wrapped_neighbor_operator<NeighborOp>::type
  derived_visitor(NeighborOp&& uf) const
  {
    return Parent::template make_wrapped_neighbor_operator<NeighborOp>::create(
      m_parent, std::forward<NeighborOp>(uf));
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
  void visit(descriptor_type const& target, NeighborOp&& neighbor_op) const
  {
    transporter_type::single(this->graph(),
      target, derived_visitor(std::forward<NeighborOp>(neighbor_op)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Visits the target vertex and apply the provided update functor,
  ///        with a given priority
  ///
  /// @deprecated Now that all visitations are executed in-place, they are
  ///             no longer scheduled and thus there is no possibility to
  ///             customize the scheduling with priorities.
  ///
  /// @ingroup pgraphAlgoDetails
  //////////////////////////////////////////////////////////////////////
  template <class NeighborOp>
  void visit(descriptor_type const& target,
             NeighborOp&& neighbor_op,
             default_info) const
  {
    stapl::abort("Priority visitation no longer supported");
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
  void visit_all_edges(Vertex source, NeighborOp&& neighbor_op) const
  {
    using has_contiguous_edges_t =
      typename has_contiguous_edges<typename std::decay<Vertex>::type>::type;

    sgl::visit_logger::visit_from(
      source, neighbor_op, this->level(), this->max_level());

    this->visit_all_edges_impl(
      std::forward<Vertex>(source),
      std::forward<NeighborOp>(neighbor_op),
      has_contiguous_edges_t{}
    );
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
  void visit_all_edges_if(Vertex&& source, NeighborOp&& neighbor_op,
                          VisitPredicate&& pred) const
  {
    using has_contiguous_edges_t =
      typename has_contiguous_edges<typename std::decay<Vertex>::type>::type;

    this->visit_all_edges_if_impl(
      std::forward<Vertex>(source),
      std::forward<NeighborOp>(neighbor_op),
      std::forward<VisitPredicate>(pred),
      has_contiguous_edges_t{}
    );
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
  void add_edge(descriptor_type const& source, descriptor_type const& target,
                edge_property const& ep = edge_property()) const
  {
    graph()->add_edge_async(source, target);
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
  void delete_edge(descriptor_type const& source,
                    descriptor_type const& target) const
  {
    graph()->delete_edge(source, target);
  }

  size_t const& level() const
  { return m_parent.level(); }

  size_t const& max_level() const
  { return m_parent.max_level(); }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of visit_all_edges when the adjacency list
  ///        is contiguous.
  /// @see visit_all_edges
  //////////////////////////////////////////////////////////////////////
  template <class Vertex, class NeighborOp>
  void visit_all_edges_impl(Vertex&& source,
                            NeighborOp&& neighbor_op,
                            std::true_type) const
  {
    using edge_iterator = decltype(source.begin());
    using edge_type = decltype(*source.begin());

    auto nop = derived_visitor(std::forward<NeighborOp>(neighbor_op));

    // Send an entire range at once
    auto visit_range =
      [this, &nop](edge_iterator begin, edge_iterator end, location_type loc) {
        transporter_type::bulk(this->graph(), loc, begin, end, nop);
      };

    // Group by home location
    auto home_of_target = [this](edge_type const& e) {
      return this->home_location(e.target());
    };

    stapl::utility::for_each_range_by(
      source.begin(), source.end(), visit_range, home_of_target);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of visit_all_edges when the adjacency list
  ///        is not contiguous.
  /// @see visit_all_edges
  //////////////////////////////////////////////////////////////////////
  template <class Vertex, class NeighborOp>
  void visit_all_edges_impl(Vertex source,
                            NeighborOp&& neighbor_op,
                            std::false_type) const
  {
    for (auto const& e : source)
      this->visit(e.target(), neighbor_op);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of visit_all_edges_if when the adjacency list
  ///        is not contiguous.
  /// @see visit_all_edges_if
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex, typename NeighborOperator, typename VisitPredicate>
  void visit_all_edges_if_impl(Vertex&& source, NeighborOperator&& neighbor_op,
                              VisitPredicate&& pred, std::false_type) const
  {
    for (auto const& e : source)
      if (pred(e))
        this->visit(e.target(), neighbor_op);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation of visit_all_edges_if when the adjacency list
  ///        is contiguous.
  /// @see visit_all_edges_if
  /// @todo The multiple calls of home_location could be avoid if the
  /// for_each_filtered_range_by function is implemented.
  //////////////////////////////////////////////////////////////////////
  template <class Vertex, class NeighborOp, typename VisitPredicate>
  void visit_all_edges_if_impl(Vertex&& source,
                               NeighborOp&& neighbor_op,
                               VisitPredicate&& pred,
                               std::true_type) const
  {
    using edge_iterator = typename std::decay<Vertex>::type::adj_edge_iterator;
    using edge_type =
      typename std::decay<Vertex>::type::adj_edges_type::value_type;

    stapl_assert(graph() != nullptr,
      "visit_all_edges_if called with null graph view pointer");

    const auto nop = derived_visitor(std::forward<NeighborOp>(neighbor_op));

    // Send an entire range at once
    auto visit_range_func = [this, &nop](edge_iterator begin,
                                         edge_iterator end) {
      const size_t loc = this->home_location((*begin).target());
      transporter_type::bulk(this->graph(), loc, begin, end, nop);
    };

    // Try to group together edge visits if they are on the same home location
    auto same_destination_comp
      = [this](edge_type const& x, edge_type const& y) {
          return this->home_location(x.target()) == home_location(y.target());
        };

    utility::for_each_filtered_range(
      source.begin(), source.end(), visit_range_func,
      same_destination_comp, pred
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Home location of a given vertex
  //////////////////////////////////////////////////////////////////////
  template<typename VertexDescriptor>
  location_type home_location(VertexDescriptor const& v) const
  { return graph()->locality(v).location(); }
};

} // namespace kla_detail

} // namespace stapl

#endif

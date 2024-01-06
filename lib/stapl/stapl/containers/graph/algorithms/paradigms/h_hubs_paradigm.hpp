/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_H_HUBS_PARADIGM_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_H_HUBS_PARADIGM_HPP

#include <stapl/containers/graph/algorithms/paradigms/h_paradigm.hpp>

namespace stapl {

namespace h_paradigm_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Work function to apply updates to hub supervertices.
//////////////////////////////////////////////////////////////////////
template<typename VD, typename UpdateFunc>
class hubs_store
{
  VD m_source;
  UpdateFunc m_uf;

public:
  typedef void result_type;

  hubs_store(VD const& source, UpdateFunc const& uf)
    : m_source(source), m_uf(uf)
  { }

  template<typename ParentVertex>
  result_type operator()(ParentVertex&& pv) const
  {
    pv.property().apply_updates_to_hubs(m_source, m_uf);
  }

  void define_type(typer& t)
  {
    t.member(m_source);
    t.member(m_uf);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to flush updated hub-representative supervertices
/// to the actual hub-vertices.
///
/// A vertex-property reducer is applied to the vertex-property of the actual
/// hub-vertex with properties from its hub-representative supervertices.
/// @tparam VPR Type of the vertex-property reducer.
//////////////////////////////////////////////////////////////////////
template<typename VPR>
class h_map_wf1
{
  VPR m_vpr;
 public:

  h_map_wf1(VPR const& vpr)
    : m_vpr(vpr)
  { }

  typedef void result_type;

  template<typename HubSuperVertex>
  result_type operator()(HubSuperVertex v)
  {
    v.property().flush_updates_to_hubs(m_vpr);
  }

  void define_type(typer& t)
  { t.member(m_vpr); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to wrap the user provided vertex-operator.
///
/// The work function is applied on a vertex if the vertex is active.
/// Active vertices may perform some computation and update their values, and
/// may visit their neighboring vertices with the user provided
/// neighbor-operator.
/// The remote neighbors are updated via the parent vertices in the
/// hierarchical and hub graphs.
/// Returns true if vertex was active (i.e. the user's work function returned
/// true), false otherwise.
/// @tparam GraphView Type of the input @ref graph_view.
/// @tparam HGraphView Type of the hierarchical machine @ref graph_view over
/// the input graph.
/// @tparam HubsGraphView Type of the hierarchical hubs @ref graph_view over
/// the input graph.
/// @tparam WF Type of the user provided vertex-operator.
/// @tparam UF The type of the user provided neighbor-operator expressing
/// computation to be performed over neighboring vertices.
/// @tparam AggrView Type of the aggregator for aggregating and storing updates.
/// @todo This work function stores a view to the hub graph that is
/// needed for calling apply_set on hub vertices inorder to apply
/// the visitor. This is currently passed in through the operator() as
/// a @ref repeat_view, with the pointer being set to the view received in
/// the operator(). However, this is slower than passing the view through
/// the constructor by ~7% on Hopper (using the g500 benchmark test). This
/// is documented in GForge to-do task #1207.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename GraphView, typename HGraphView, typename HubsGraphView,
         typename WF, typename UF, typename AggrView>
class h_hubs_map_wf0
  : public h_map_wf0<GraphView, HGraphView, WF, UF, AggrView>
{
  typedef typename GraphView::vertex_descriptor    descriptor_t;
  typedef typename HGraphView::view_container_type hg_cont_t;

  HubsGraphView*  m_hubs_vw;

  typedef h_map_wf0<GraphView, HGraphView, WF, UF, AggrView> base_type;

 public:

  typedef typename base_type::result_type result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param gvw The @ref graph_view over the input graph.
  /// @param hvw The hierarchical machine @ref graph_view over the input graph.
  /// @param hubs The hierarchical hubs @ref graph_view over the input graph.
  /// @param aggr_vw The aggregator for aggregating and storing updates.
  /// @param wf The user provided vertex-operator expressing computation to be
  /// performed over each vertex.
  /// @param curr_level The current level of this visit.
  //////////////////////////////////////////////////////////////////////
  h_hubs_map_wf0(GraphView& gvw, HGraphView& hvw, HubsGraphView& hubs,
                 AggrView& aggr_vw,
                 WF const& wf, size_t curr_level)
    : base_type(gvw, hvw, aggr_vw, wf, curr_level),
      m_hubs_vw(&hubs)
  { }

  template<typename Vertex>
  result_type operator()(Vertex&& v)
  {
    return this->m_wf(std::forward<Vertex>(v), *this);
  }

  //////////////////////////////////////////////////////////////////////
  /// Allows an update @param uf to be sent to hub neighbors of
  /// @p source through a hub-representative vertex in the next level
  /// of the hierarchy.
  /// @param source The source vertex whose neighbors are being visited.
  /// @param uf The visitor to be applied on the target vertex.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex, typename UpdateFunc>
  void visit_hubs(Vertex const& source, UpdateFunc const& uf)
  {
    // Store outgoing messages to remote locations in the upper-level.
     m_hubs_vw->apply_set(this->m_location_id,
       hubs_store<descriptor_t, UpdateFunc>(source.descriptor(), uf));
  }

  //////////////////////////////////////////////////////////////////////
  /// Provides a visit_all_edges() helper to user's work function
  /// so they can visit all neighboring vertices without exposing internals.
  /// @param source The source vertex whose neighbors are being visited.
  /// @param uf The visitor to be applied on the target vertex.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex, typename UpdateFunc>
  void visit_all_edges(Vertex source, UpdateFunc const& uf)
  {
    // Inform the parent vertex of this update, so it can send the non-local
    // updates.
    this->visit_parent(source, uf);

    // Inform the hub-representative vertex of this update, so it can send
    // the update to neighboring hub-vertices.
    this->visit_hubs(source, uf);

    // Only visit local vertices that are not hubs, any non-local or hub edges
    // should be deleted by @ref create_level_hubs() and
    // @ref create_level_machine().
    for (auto const& e : source) {
      this->m_gvw->container().apply_set(e.target(), uf);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Degree of vertex v. Takes into account edges that were cut
  ///        in the lower graph and the hub graph.
  //////////////////////////////////////////////////////////////////////
  template<typename V>
  std::size_t degree(V&& v) const
  {
    auto sv = m_hubs_vw->operator[](this->m_location_id).property();
    const std::size_t degree_to_hubs = sv.cut_edges_size(v.descriptor());

    return base_type::degree(std::forward<V>(v)) + degree_to_hubs;
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_hubs_vw);
  }
};

} // namespace h_paradigm_impl


//////////////////////////////////////////////////////////////////////
/// @brief The Parallel Hierarchical Hubs Level-Synchronous (h-hubs) Paradigm.
///
/// Implements the hierarchical hubs level-sync paradigm, which iteratively
/// executes BSP-Supersteps (SS). Each SS applies the user provided
/// vertex-operator over the vertices of the input graph.
/// Any communication generated within a SS is guaranteed to have finished
/// before the SS finishes.
/// The communication happens hierarchically, by following the locality
/// of the graph through the machine-hierarchy and the hubs-hierarchy.
/// Updates (neighbor-operators) to be applied to remote neighbors of a
/// vertex v are sent to v's parent vertex in the machine hierarchy. The
/// parent then forwards the update to all its neighbors that contain neighbors
/// of v. When the update is received by the parent's neighbor, it applies the
/// update to all of its children that are neighbors of v in the lower-level
/// graph.
/// The update is also sent to v's parent vertex in its hub-hierarchy. The
/// hub-hierarchy contains representatives of high-degree vertices (hubs) on
/// each location, created using @ref create_level_hubs(). The update is
/// applied to the hub-representatives of all neighboring hub-vertices of vertex
/// v. Upon completion of an SS, the hub-representatives on each location are
/// flushed to the actual hubs vertices in the lower-level graph, using the
/// provided reduction operator (@p vpr).
/// Finally, the local neighbors of v are updated.
/// This allows us to send a single update per remote location vs. sending
/// an update per remote/cross edge, as well as avoid heavy communication to
/// hub-vertices. This may lead to a reduction in the amount of communication
/// from O(E) to O(V), providing faster and more scalable performance.
///
/// The hierarchical hubs paradigm is an extension of the hierarchical paradigm
/// (@ref h_paradigm()), and adds the ability to reduce communication to hub
/// vertices of the graph. The hierarchical paradigm reduces outgoing
/// communication from vertices, while the hubs-hierarchy allows the reduction
/// of incoming communication to high-degree vertices.
///
/// The user provides a vertex-operator to express the computation to be
/// performed on each vertex, and a neighbor-operator that will be applied
/// to each neighbor that is visited.
/// The vertex-operator is passed in a vertex and a visit object. To visit a
/// neighboring vertex, the vertex-operator must call
/// visit_all_edges(neighbor, visitor()).
/// The vertex-operator must return true if the vertex was active (i.e. its
/// value was updated), or false otherwise.
/// The neighbor-operator is passed in the target vertex. Neighbor-operators
/// may carry state, but must be immutable. They should return true if the
/// visit was successful (i.e. the target vertex will be activated after this
/// visit), or false otherwise.
/// Users may also provide additional functions to be executed before and after
/// each SS.
/// @tparam WF The type of the user provided vertex-operator expressing
/// computation to be performed over each vertex.
/// @tparam UF The type of the user provided neighbor-operator expressing
/// computation to be performed over neighboring vertices.
/// @param vpr A vertex property reducer for the algorithm. It should accept
/// two vertex properties and reduce them to update the first one. Used to
/// update the hub vertices.
/// @param post_execute Optional functor that will be executed on the
/// @ref graph_view at the end of each SS. This will be invoked with the
/// input @ref graph_view and the current SS ID (the ID of the SS that
/// just finished).
/// @param h The hierarchical machine @ref graph_view over the input graph.
/// This must be created using @ref create_level_machine().
/// @param hubs The hierarchical hubs @ref graph_view over the input graph.
/// This must be created using @ref create_level_hubs() over the input graph,
/// before calling @ref create_level_machine().
/// @param g The @ref graph_view over the input graph.
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename WF, typename UF, typename VPR, typename PostExecute,
         typename HView, typename HubsView, typename GView>
size_t h_hubs_paradigm(WF const& uwf, UF const&, VPR const& vpr,
                       PostExecute post_execute,
                       HView& h, HubsView& hubs, GView& g)
{
  // Sort all edges for each vertex according to their target home-locations.
  // This may help increase performance through aggregation of communication.
  g.sort_edges();

  typedef typename GView::vertex_descriptor vd_t;
  typedef typename HView::view_container_type cont_t;
  typedef h_paradigm_impl::aggr_func<vd_t, UF> aggr_func_t;
  typedef tunnel_aggregator<cont_t, aggr_func_t> aggr_t;
  aggr_t aggr(h.container());

  h_paradigm_impl::h_hubs_map_wf0<GView, HView, HubsView, WF, UF, aggr_t>
    wf0(g, h, hubs, aggr, uwf, 1);

  h_paradigm_impl::h_map_wf1<VPR> wf1(vpr);

  size_t iterations = 0;
  while (map_reduce(wf0, plus<bool>(), g)) {
    aggr.flush();

    map_func(wf1, hubs);

    post_execute(g, iterations);
    //now change the propagation trigger
    wf0.increment_iteration();
    ++iterations;
  }
  return iterations;
}


//////////////////////////////////////////////////////////////////////
/// @brief The Parallel Hierarchical Hubs Level-Synchronous (h-hubs) Paradigm.
///
/// Overloaded variant of @ref h_hubs_paradigm() with an empty post-execute.
//////////////////////////////////////////////////////////////////////
template<typename WF, typename UF, typename VPR,
         typename HView, typename HubsView, typename GView>
size_t h_hubs_paradigm(WF const& uwf, UF const&, VPR const& vpr,
                       HView& h, HubsView& hubs, GView& g)
{
  return h_hubs_paradigm(uwf, UF(), vpr,
                         kla_detail::empty_prepost_execute(), h, hubs, g);
}

} // namespace stapl

#endif

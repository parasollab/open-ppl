/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_H2_PARADIGM_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_H2_PARADIGM_HPP

#include <stapl/containers/graph/algorithms/aggregator.hpp>
#include <stapl/containers/graph/algorithms/create_level_machine.hpp>
#include <stapl/containers/graph/algorithms/paradigms/kla_paradigm.hpp>
#include <stapl/containers/graph/algorithms/paradigms/h_paradigm.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/paragraph/paragraph.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/runtime.hpp>

namespace stapl {

namespace h_paradigm_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to visit the parent vertex with the aggregated
/// neighbor-operator.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct parent_process_wf
{
  size_t m_gid;

  typedef void result_type;

  parent_process_wf(size_t gid)
    : m_gid(gid)
  { }

  template<typename Container, typename Elem>
  result_type operator()(Container* c, Elem&& e) const
  { c->apply_set(m_gid, std::forward<Elem>(e)); }

  void define_type(typer& t)
  { t.member(m_gid); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work function to wrap the user provided vertex-operator.
/// The work function is applied on a vertex if the vertex is active.
/// Active vertices may perform some computation and update their values, and
/// may visit their neighboring vertices with the user provided
/// neighbor-operator.
/// The remote neighbors are updated via the parent vertices in the
/// hierarchical graph.
/// Returns true if vertex was active (i.e. the user's work function returned
/// true), false otherwise.
/// @tparam GraphView Type of the input @ref graph_view.
/// @tparam H1GraphView Type of the hierarchical machine @ref graph_view over
/// the input graph (1st level).
/// @tparam H2GraphView Type of the hierarchical machine @ref graph_view over
/// the input graph (2nd level).
/// @tparam WF Type of the user provided vertex-operator.
/// @tparam UF The type of the user provided neighbor-operator expressing
/// computation to be performed over neighboring vertices.
/// @tparam Aggr1View Type of the aggregator for aggregating and storing
/// updates for level-1 messages.
/// @tparam Aggr2View Type of the aggregator for aggregating and storing
/// updates for level-2 messages.
/// @tparam TranslatorFunc The mapping of physical location to hierarchical
/// location/vertex.
/// @ingroup pgraphAlgoDetails
/// @todo This work function stores a view to the input graph that is
/// needed for calling apply_set on target vertices in order to apply
/// the visitor. This is currently passed in through the operator() as
/// a repeat_view(), with the pointer being set to the view received in
/// the operator(). However, this is slower than passing the view through
/// the constructor by ~7% on Hopper (using the g500 benchmark test). This
/// is documented in GForge to-do task #1207.
//////////////////////////////////////////////////////////////////////
template<typename GraphView, typename H1GraphView, typename H2GraphView,
         typename WF, typename UF, typename Aggr1View, typename Aggr2View,
         typename TranslatorFunc>
class h_map_wf2
  : public h_map_wf0<GraphView, H1GraphView, WF, UF, Aggr1View>
{
protected:
  typedef h_map_wf0<GraphView, H1GraphView, WF, UF, Aggr1View> base_type;
  typedef typename GraphView::vertex_descriptor    descriptor_t;

  size_t         m_upper_location_id;
  H2GraphView*   m_h2vw;

  typedef aggr_func<descriptor_t, UF>            aggr_func_t;
  typedef aggr_func<descriptor_t, aggr_func_t>   aggr2_func_t;
  typedef parent_store<aggr2_func_t, Aggr2View>  parent_store2_wf_t;

  Aggr2View* m_aggr2_vw;

  size_t m_filled, m_aggr_sz;
  typedef std::vector<parent_store2_wf_t> send_aggr_t;
  send_aggr_t m_send_aggr;
  parent_process_wf m_parent_process_wf;

 public:
  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param gvw The @ref graph_view over the input graph.
  /// @param hvw The hierarchical machine @ref graph_view over the input graph.
  /// @param aggr_vw The aggregator for aggregating and storing updates.
  /// @param wf The user provided vertex-operator expressing computation to be
  /// performed over each vertex.
  /// @param curr_level The current level of this visit.
  //////////////////////////////////////////////////////////////////////
  h_map_wf2(GraphView& gvw, H1GraphView& h1vw, H2GraphView& h2vw,
            Aggr1View& aggr1_vw, Aggr2View& aggr2_vw, WF const& wf,
            TranslatorFunc const& translator, size_t curr_level,
            size_t aggr_sz = 512)
    : base_type(gvw, h1vw, aggr1_vw, wf, curr_level),
      m_upper_location_id(translator(gvw.get_location_id())),
      m_h2vw(&h2vw),
      m_aggr2_vw(&aggr2_vw),
      m_filled(0),
      m_aggr_sz(aggr_sz),
      m_send_aggr(aggr_sz),
      m_parent_process_wf(m_upper_location_id)
  { }

  template<typename Vertex>
  result_type operator()(Vertex v)
  {
    return this->m_wf.template operator()<Vertex, h_map_wf2&>(v, *this);
  }

  //////////////////////////////////////////////////////////////////////
  /// Allows an update @param uf to be sent to non-local neighbors of
  /// @param source through its parent vertex in the next level of the
  /// hierarchy.
  /// @param source The source vertex whose neighbors are being visited.
  /// @param uf The visitor to be applied on the target vertex.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex, typename UpdateFunc>
  void visit_parent2(Vertex const& source, UpdateFunc const& uf)
  {
    if (m_filled == m_aggr_sz) {
      /// Store outgoing messages to remote locations in the upper-level.
      m_h2vw->container().aggregate_async(m_upper_location_id, m_send_aggr,
                                          m_parent_process_wf);
      m_filled = 0;
    }
    m_send_aggr[m_filled++]
      = parent_store2_wf_t(aggr2_func_t(this->m_location_id,
                                        aggr_func_t(source.descriptor(), uf)),
                           m_aggr2_vw);
  }

  //////////////////////////////////////////////////////////////////////
  /// Provides a visit_all_edges() helper to user's work function
  /// so they can visit all neighboring vertices without exposing internals.
  /// @param source The source vertex whose neighbors are being visited.
  /// @param uf The visitor to be applied on the target vertex.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex, typename UpdateFunc>
  void visit_all_edges(Vertex const& source, UpdateFunc const& uf)
  {
    // Inform the 2nd-level parent vertex of this message, so it can send the
    // non-local updates.
    visit_parent2(source, uf);

    // Inform the 1st-level parent vertex of this message, so it can send the
    // non-local updates.
    this->visit_parent(source, uf);

    // Only visit local vertices, any non-local edges should be deleted
    // by @ref create_level_machine().
    this->visit_local_neighbors(source, uf);
  }

  ~h_map_wf2(void)
  {
    if (m_filled > 0)
      m_h2vw->container().aggregate_async(
        m_upper_location_id,
        send_aggr_t(m_send_aggr.begin(), m_send_aggr.begin()+m_filled),
        m_parent_process_wf);
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_upper_location_id);
    t.member(m_h2vw);
    t.member(m_aggr2_vw);
    t.member(m_filled);
    t.member(m_aggr_sz);
    t.member(m_send_aggr);
    t.member(m_parent_process_wf);
  }
};

} // namespace h_paradigm_impl


//////////////////////////////////////////////////////////////////////
/// @brief The Parallel 2-level Hierarchical Level-Synchronous (h2) Paradigm.
///
/// Implements the hierarchical level-sync paradigm, which iteratively
/// executes BSP-Supersteps (SS). Each SS applies the user provided
/// vertex-operator over the vertices of the input graph.
/// Any communication generated within a SS is guaranteed to have finished
/// before the SS finishes.
/// The communication happens hierarchically, by following the locality
/// of the graph through the machine-hierarchy. Updates (neighbor-operators)
/// to be applied to remote neighbors of a vertex v are sent to v's parent
/// vertex in the machine hierarchy. The parent then forwards the update to
/// all its neighbors that contain neighbors of v. When the update is received
/// by the parent's neighbor, it applies the update to all of its children that
/// are neighbors of v in the lower-level graph. This allows us to send a single
/// update per remote location vs. sending an update per remote/cross edge.
/// Next, the local neighbors of v are updated.
/// This may lead to a reduction in the amount of communication from O(E) to
/// O(V), providing faster and more scalable performance.
///
/// The user provides a vertex-operator to express the computation to be
/// performed on each vertex, and a neighbor-operator that will be applied
/// to each neighbor that is visited.
/// The vertex-operator is passed in a vertex and a visit object. To visit a
/// neighboring vertex, the vertex-operator must call
/// visit_all_edges(neighbor, neighbor_operator()).
/// The vertex-operator must return true if the vertex was active (i.e. its
/// value was updated), or false otherwise.
/// The neighbor-operator should be instantiated within vertex-operators and
/// passed to a visit method to be applied to a vertex's neighbors. It is passed
/// in a neighboring vertex. Neighbor-operators instantiated within
/// vertex-operators may carry state, but must be immutable. They should return
/// true if the visit was successful (i.e. the target vertex will be activated
/// after this visit), or false otherwise.
/// Users may also provide additional functions to be executed before and after
/// each SS.
/// @tparam WF The type of the user provided vertex-operator expressing
/// computation to be performed over each vertex.
/// @tparam UF The type of the user provided neighbor-operator expressing
/// computation to be performed over neighboring vertices.
/// @param post_execute Optional functor that will be executed on the
/// @ref graph_view at the end of each SS. This will be invoked with the
/// input @ref graph_view and the current SS ID (the ID of the SS that
/// just finished).
/// @param h2 The hierarchical machine @ref graph_view over the input graph.
/// This must be created using @ref create_level2_machine().
/// @param h1 The hierarchical machine @ref graph_view over the input graph.
/// This must be created using @ref create_level_machine().
/// @param g The @ref graph_view over the input graph.
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename WF, typename UF, typename PostExecute,
         typename H2View, typename H1View, typename GView>
size_t h2_paradigm(WF const& uwf, UF const&, PostExecute post_execute,
                   H2View& h2, H1View& h1, GView& g, size_t aggr_sz = 512)
{
  // Sort all edges for each vertex according to their target home-locations.
  // This may help increase performance through aggregation of communication.
  g.sort_edges();

  typedef typename GView::vertex_descriptor vertex_desc_t;
  typedef typename H1View::view_container_type h1view_cont_t;
  typedef typename H2View::view_container_type h2view_cont_t;
  typedef h_paradigm_impl::aggr_func<vertex_desc_t, UF> aggr_func_t;
  typedef h_paradigm_impl::aggr_func<vertex_desc_t, aggr_func_t> aggr2_func_t;
  typedef tunnel_aggregator<h1view_cont_t, aggr_func_t> aggr1_t;
  typedef tunnel_aggregator<h2view_cont_t, aggr2_func_t> aggr2_t;
  aggr1_t aggr1(h1.container());
  aggr2_t aggr2(h2.container());

  const size_t num_locs_per_process
    = runtime::this_context::get().get_gang_md().local_size();

  h_paradigm_impl::h_map_wf2<GView, H1View, H2View, WF, UF, aggr1_t, aggr2_t,
                             create_level_machine_detail::translator>
    wf0(g, h1, h2, aggr1, aggr2, uwf,
        create_level_machine_detail::translator(num_locs_per_process), 1,
        aggr_sz);

  size_t iterations = 0;
  while (map_reduce(wf0, plus<bool>(), g)) {
    aggr2.flush();
    aggr1.flush();

    post_execute(g, iterations);
    //now change the propagation trigger
    wf0.increment_iteration();
    ++iterations;
  }
  return iterations;
}


//////////////////////////////////////////////////////////////////////
/// @brief The Parallel 2-level Hierarchical Level-Synchronous (h2) Paradigm.
///
/// Overloaded variant of @ref h2_paradigm() with an empty post-execute.
//////////////////////////////////////////////////////////////////////
template<typename WF, typename UF,
         typename H2View, typename H1View, typename GView>
size_t h2_paradigm(WF const& uwf, UF const&, H2View& h2, H1View& h1, GView& g,
                   size_t aggr_sz = 512)
{
  return h2_paradigm(uwf, UF(),
                     kla_detail::empty_prepost_execute(), h2, h1, g,
                     aggr_sz);
}

} // namespace stapl

#endif

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_H_PARADIGM_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_H_PARADIGM_HPP

#include <stapl/containers/graph/algorithms/paradigms/kla_paradigm.hpp>
#include <stapl/containers/graph/algorithms/aggregator.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/paragraph/paragraph.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/runtime.hpp>

namespace stapl {

namespace h_paradigm_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to visit the target vertex's parent vertex with the
/// user provided neighbor-operator. Implements the hierarchical-visit
/// pattern, which applies the user's neighbor-operator via the
/// hierarchical views.
/// @tparam VD Type of the graph's vertex descriptor.
/// @tparam UpdateFunc Type of the user provided neighbor-operator.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename VD, typename UpdateFunc>
class aggr_func
{
  VD m_source;
  UpdateFunc m_update;

public:
  typedef void result_type;

  aggr_func(VD const& source, UpdateFunc const& update)
    : m_source(source), m_update(update)
  { }

  aggr_func(void) = default;

  template<typename ParentVertex>
  result_type operator()(ParentVertex&& pv) const
  {
    // Apply update-functor to all (child) targets of the
    // (child) source vertex. Information about the child targets
    // is stored on the supervertex.
    pv.property().apply_updates_to_child_targets(m_source, m_update);
  }

  VD source(void) const
  { return m_source; }

  void define_type(typer& t)
  {
    t.member(m_source);
    t.member(m_update);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to visit the parent vertex with the user provided
/// neighbor-operator. Implements the hierarchical-visit pattern, which
/// sends the user's neighbor-operator via the hierarchical views.
/// @tparam AggrUpdateFunc Type of the aggregator functor that wraps
/// the user provided neighbor-operator.
/// @tparam View Type of the aggregator.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename AggrUpdateFunc, typename View>
class parent_store
{
  AggrUpdateFunc m_apply_func;
  View*  m_view;

public:
  typedef void result_type;

  parent_store(AggrUpdateFunc const& apply_func, View* view)
    : m_apply_func(apply_func), m_view(view)
  { }

  parent_store(void) = default;

  template<typename ParentVertex>
  result_type operator()(ParentVertex&& pv) const
  {
    // Apply update-functor to neighbors of parent vertex of the source
    // who are parents of the source's targets.
    pv.property().neighbors_apply(m_apply_func.source(), m_apply_func, m_view);
  }

  void define_type(typer& t)
  { t.member(m_apply_func); }
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
/// @tparam HGraphView Type of the hierarchical machine @ref graph_view over
/// the input graph.
/// @tparam WF Type of the user provided vertex-operator.
/// @tparam UF The type of the user provided neighbor-operator expressing
/// computation to be performed over neighboring vertices.
/// @tparam AggrView Type of the aggregator for aggregating and storing updates.
/// @ingroup pgraphAlgoDetails
/// @todo This work function stores a view to the input graph that is
/// needed for calling apply_set on target vertices inorder to apply
/// the visitor. This is currently passed in through the operator() as
/// a repeat_view(), with the pointer being set to the view received in
/// the operator(). However, this is slower than passing the view through
/// the constructor by ~7% on Hopper (using the g500 benchmark test). This
/// is documented in GForge to-do task #1207.
//////////////////////////////////////////////////////////////////////
template<typename GraphView, typename HGraphView, typename WF, typename UF,
         typename AggrView>
class h_map_wf0
{
protected:
  typedef typename GraphView::vertex_descriptor    descriptor_t;

  size_t       m_curr_level;
  size_t       m_location_id;
  WF           m_wf;
  GraphView*   m_gvw;
  HGraphView*  m_hvw;
  /// True if this location has no cut edges
  bool m_no_cut_edges;

  typedef aggr_func<descriptor_t, UF> aggr_func_t;
  typedef parent_store<aggr_func_t, AggrView>  parent_store_wf_t;

  AggrView* m_aggr_vw;


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
  h_map_wf0(GraphView& gvw, HGraphView& hvw, AggrView& aggr_vw, WF const& wf,
            size_t curr_level)
    : m_curr_level(curr_level),
      m_location_id(gvw.get_location_id()), m_wf(wf), m_gvw(&gvw), m_hvw(&hvw),
      m_aggr_vw(&aggr_vw)
  {
    // If this location's supernode has no neighbors
    m_no_cut_edges =
      m_hvw->operator[](m_location_id).property().neighbors_size() == 0;
  }

  template<typename Vertex>
  result_type operator()(Vertex&& v)
  {
    return m_wf(std::forward<Vertex>(v), *this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Degree of vertex v. Takes into account edges that were cut
  ///        in the lower graph.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex>
  std::size_t degree(Vertex const& v) const
  {
    if (m_no_cut_edges)
      return v.size();

    auto sv = m_hvw->operator[](this->m_location_id).property();
    const std::size_t cut_degree = sv.cut_edges_size(v.descriptor());
    return v.size() + cut_degree;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Allows an update @c uf to be sent to non-local neighbors
  /// of @c source through its parent vertex in the next level of the
  /// hierarchy.
  /// @param source The source vertex whose neighbors are being visited.
  /// @param uf The visitor to be applied on the target vertex.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex, typename UpdateFunc>
  void visit_parent(Vertex const& source, UpdateFunc const& uf)
  {
    if (m_no_cut_edges)
      return;

    /// Store outgoing messages to remote locations in the upper-level.
    m_hvw->apply_set(m_location_id,
                     parent_store_wf_t(aggr_func_t(source.descriptor(), uf),
                     m_aggr_vw));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Allows an update @c uf to be applied to local neighbors
  /// of the @c source.
  ///
  /// @param source The source vertex whose neighbors are being visited.
  /// @param uf The visitor to be applied on the target vertex.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex, typename UpdateFunc>
  void visit_local_neighbors(Vertex source, UpdateFunc const& uf,
                             typename boost::disable_if_c<
                               boost::is_same<
                                 typename Vertex::adj_edge_iterator,
                                 typename std::vector<
                                   typename std::iterator_traits<
                                     typename Vertex::adj_edge_iterator
                                   >::value_type
                                 >::iterator>::value>::type* =0)
  {
    // Only visit local vertices, any non-local edges should be deleted
    // by @ref create_level_machine().
    for (auto && e : source) {
      m_gvw->container().apply_set(e.target(), uf);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Allows an update @c uf to be applied to local neighbors
  /// of the @c source.
  ///
  /// Specialization for when the adjacency-list is stored as a vector,
  /// providing contiguous memory-access.
  /// @param source The source vertex whose neighbors are being visited.
  /// @param uf The visitor to be applied on the target vertex.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex, typename UpdateFunc>
  void visit_local_neighbors(Vertex source, UpdateFunc const& uf,
                             typename boost::enable_if_c<
                               boost::is_same<
                                 typename Vertex::adj_edge_iterator,
                                 typename std::vector<
                                   typename std::iterator_traits<
                                     typename Vertex::adj_edge_iterator
                                   >::value_type
                                 >::iterator>::value>::type* =0)
  {
    // Only visit local vertices, any non-local edges should be deleted
    // by @ref create_level_machine().
    m_gvw->container().aggregate_apply_async(m_gvw->get_location_id(),
                                             make_range(source.begin(),
                                                        source.end()), uf);
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
    // Inform the parent vertex of this message, so it can send the non-local
    // updates.
    visit_parent(source, uf);

    // Only visit local vertices, any non-local edges should be deleted
    // by @ref create_level_machine().
    this->visit_local_neighbors(source, uf);
  }


  size_t level() const
  { return m_curr_level; }

  void increment_iteration()
  { ++m_curr_level; }

  void define_type(typer& t)
  {
    t.member(m_curr_level);
    t.member(m_wf);
    t.member(m_gvw);
    t.member(m_hvw);
    t.member(m_no_cut_edges);
  }
};

} // namespace h_paradigm_impl


//////////////////////////////////////////////////////////////////////
/// @brief The Parallel Hierarchical Level-Synchronous (h) Paradigm.
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
/// @param h The hierarchical machine @ref graph_view over the input graph.
/// This must be created using @ref create_level_machine().
/// @param g The @ref graph_view over the input graph.
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename WF, typename UF, typename PostExecute,
         typename HView, typename GView>
size_t h_paradigm(WF const& uwf, UF const&, PostExecute post_execute,
                  HView& h, GView& g)
{
  // Sort all edges for each vertex according to their target home-locations.
  // This may help increase performance through aggregation of communication.
  g.sort_edges();

  typedef typename GView::vertex_descriptor vertex_desc_t;
  typedef typename HView::view_container_type hview_cont_t;
  typedef h_paradigm_impl::aggr_func<vertex_desc_t, UF> aggr_func_t;
  typedef tunnel_aggregator<hview_cont_t, aggr_func_t> aggr_t;
  aggr_t aggr(h.container());

  h_paradigm_impl::h_map_wf0<GView, HView, WF, UF, aggr_t> wf0(g, h, aggr,
                                                               uwf, 1);

  size_t iterations = 0;
  while (map_reduce(wf0, plus<bool>(), g)) {
    aggr.flush();

    post_execute(g, iterations);
    //now change the propagation trigger
    wf0.increment_iteration();
    ++iterations;
  }
  return iterations;
}


//////////////////////////////////////////////////////////////////////
/// @brief The Parallel Level-Synchronous (H) Paradigm.
///
/// Overloaded variant of @ref h_paradigm() with an empty post-execute.
//////////////////////////////////////////////////////////////////////
template<typename WF, typename UF, typename HView, typename GView>
size_t h_paradigm(WF const& uwf, UF const&, HView& h, GView& g)
{
  return h_paradigm(uwf, UF(),
                    kla_detail::empty_prepost_execute(), h, g);
}

} // namespace stapl

#endif

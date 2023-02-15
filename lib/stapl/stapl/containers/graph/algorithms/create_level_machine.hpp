/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_CREATE_LEVEL_MACHINE_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_CREATE_LEVEL_MACHINE_HPP

#include <stapl/containers/graph/digraph.hpp>
#include <stapl/containers/graph/algorithms/aggregator.hpp>
#include <stapl/containers/graph/csr_utils.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/views/lazy_graph_view.hpp>
#include <stapl/domains/interval.hpp>
#include <unordered_map>
#include <unordered_set>

namespace stapl {

namespace create_level_machine_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex property class for the @ref create_level_machine algorithm.
/// @tparam GraphView The type of the lower-level graph.
/// @ingroup pgraphAlgo
///
/// This property provides functionality needed to support a hierarchy of
/// levels.
//////////////////////////////////////////////////////////////////////
template<typename GraphView>
struct super_vertex_machine_property
{
  typedef typename GraphView::vertex_descriptor vd_type;
  typedef std::unordered_map<vd_type, std::vector<vd_type> > child_cont_type;
  typedef std::unordered_map<vd_type,
                             std::unordered_set<vd_type> >   neighbor_cont_type;

  using degrees_map_type = std::unordered_map<vd_type, std::size_t>;

  /// Stores the vertex descriptors of children vertices.
  child_cont_type     m_children;
  /// @brief Stores the vertex descriptors of children vertices
  /// which are neighbors of the incoming source vertex.
  neighbor_cont_type  m_neighbors;
  /// Map to store the number of cut edges for vertices in the lower graph
  degrees_map_type m_cut_degrees;
  /// Stores the lower-level input graph view.
  GraphView*          m_lower_level;

  super_vertex_machine_property(void)
    : m_children(), m_neighbors(), m_lower_level(0)
  { }

  super_vertex_machine_property(GraphView& g)
    : m_children(), m_neighbors(), m_lower_level(&g)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the provided functor to children vertices of this vertex
  /// in the lower-level graph that are targets of edges from the given source.
  /// (local).
  /// @param source The descriptor of the source vertex (from lower-level
  /// graph). The target vertices in the lower-level graph owned by the current
  /// supervertex will have the functor applied to them.
  /// @param f The functor to be applied to the child targets.
  //////////////////////////////////////////////////////////////////////
  template<typename Func>
  void apply_updates_to_child_targets(vd_type const& source, Func const& f)
  {
    auto it = m_children.find(source);
    if (it != m_children.end())
      for (auto const& child_target : it->second)
        m_lower_level->container().apply_set(child_target, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a child vertex to this supervertex.
  /// @param source The descriptor of the source vertex (from lower graph).
  /// @param i The descriptor of the child vertex of this supervertex.
  //////////////////////////////////////////////////////////////////////
  void add_child(vd_type const& source, vd_type const& i)
  { m_children[source].push_back(i); }


  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a parent neighbor to given child vertex of this supervertex.
  /// @param child_source The descriptor of the child vertex (from lower graph).
  /// @param parent_target The descriptor of the child's neighbor's supervertex.
  //////////////////////////////////////////////////////////////////////
  void add_child_neighbor(vd_type const& child_source,
                          vd_type const& parent_target)
  { m_neighbors[child_source].insert(parent_target); }


  //////////////////////////////////////////////////////////////////////
  /// @brief Removes the parent neighbor from all children of this supervertex.
  /// @param parent_target The descriptor of the child's neighbor's supervertex.
  //////////////////////////////////////////////////////////////////////
  void remove_child_neighbor(vd_type const& parent_target)
  {
    for (auto& x : m_neighbors) {
      auto it = std::find(x.second.begin(), x.second.end(), parent_target);
      if (it != x.second.end())
        x.second.erase(it);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Increments the number of cut edges for a vertex in the lower graph
  /// @param vd A vertex in the lower graph
  /// @param degree The amount to increment the edge cut
  //////////////////////////////////////////////////////////////////////
  void add_cut_degree(vd_type const& vd, std::size_t degree)
  {
    m_cut_degrees[vd] += degree;
  }

  template<typename Func, typename View>
  void neighbors_apply(vd_type const& child_source, Func const& f, View* vw)
  {
    /// Apply the update functor on all neighbors of parent of source
    /// which contain neighbors of source.
    for (auto const& t : m_neighbors[child_source])
      vw->apply_set(t, f);
  }

  size_t children_size(void) const
  { return m_children.size(); }

  size_t neighbors_size(void) const
  { return m_neighbors.size(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Retrieve the size of the cut edges for a single vertex in the
  ///        lower graph
  /// @param vd A vertex in the lower graph
  //////////////////////////////////////////////////////////////////////
  std::size_t cut_edges_size(vd_type const& vd) const
  {
    auto it = m_cut_degrees.find(vd);
    return it == m_cut_degrees.end() ? 0 : it->second;
  }

  void define_type(typer& t)
  {
    t.member(m_lower_level);
    abort("super_vertex_machine_property: Incorrect define_type()");
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to add a child source vertex and its target to
/// a supervertex.
/// @tparam VD Type of the vertex-descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VD>
struct add_child_edge_to_target_sv_wf
{
  VD m_target_sv;
  VD m_child_source;
  VD m_child_target;

  add_child_edge_to_target_sv_wf(VD const& target_sv, VD const& child_source,
                                 VD const& child_target)
    : m_target_sv(target_sv), m_child_source(child_source),
      m_child_target(child_target)
  { }

  VD target(void) const
  { return m_target_sv; }

  template<typename SuperVertexProperty>
  void operator() (SuperVertexProperty& svp) const
  { svp.add_child(m_child_source, m_child_target); }

  void define_type(typer& t)
  {
    t.member(m_target_sv);
    t.member(m_child_source);
    t.member(m_child_target);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to add a child source vertex to a superedge
/// property.
/// @tparam VD Type of the vertex-descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename VD>
struct sv_edge_property_add_child_wf
{
  VD m_child_source;
  VD m_parent_target;

  typedef void result_type;

  sv_edge_property_add_child_wf(VD const& child_source, VD const& parent_target)
    : m_child_source(child_source), m_parent_target(parent_target)
  { }

  template<typename SuperEdgeProperty>
  result_type operator()(SuperEdgeProperty& sep) const
  { sep.add_child_neighbor(m_child_source, m_parent_target); }

  void define_type(typer& t)
  {
    t.member(m_child_source);
    t.member(m_parent_target);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to store in a supervertex the cut degree for
///        each vertex in the lower graph.
/// @tparam HGraph The hierarchical graph.
/// @tparam LocalityFunc Predicate used to determine if an edge is a cut edge
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename HGraph, typename LocalityFunc>
struct add_cut_degree_wf
{
  mutable HGraph* m_h_graph;
  LocalityFunc m_locality_func;
  location_type m_loc;

  add_cut_degree_wf(HGraph* h_graph, LocalityFunc locality_func)
    : m_h_graph(h_graph), m_locality_func(std::move(locality_func)),
      m_loc(h_graph->get_location_id())
  { }

  using result_type = void;

  template<typename V>
  result_type operator()(V&& v) const
  {
    std::size_t cut_degree = std::count_if(v.begin(), v.end(), m_locality_func);
    m_h_graph->operator[](m_loc).property().add_cut_degree(
      v.descriptor(), cut_degree
    );
  }

  void define_type(typer& t)
  {
    t.member(m_h_graph);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Predicate to determine if a GID is local to the current location.
/// @tparam G The type of the container.
/// @tparam H The type of the hierarchical view.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename G, typename H>
struct is_local_0
{
  using vertex_descriptor = typename H::vertex_descriptor;
  G* m_gll;

  is_local_0(G* gll)
    : m_gll(gll)
  { }

  bool operator()(vertex_descriptor const& gid) const
  { return m_gll->is_local(gid); }

  template<typename Edge>
  bool operator()(Edge const& e) const
  {
   return !m_gll->is_local(e.target());
  }

  void define_type(typer& t)
  { t.member(m_gll); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Predicate to determine if a GID is local to the current hierarchical
/// location/vertex, as given by a provided translation mapping.
/// @tparam G The type of the container.
/// @tparam TranslatorFunc The mapping of physical location to hierarchical
/// location/vertex.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename G, typename TranslatorFunc>
struct is_local_1
{
  typedef typename G::vertex_descriptor vertex_descriptor;
  G* m_g;
  TranslatorFunc m_translator;
  location_type  m_location_id;

  is_local_1(G* g, TranslatorFunc const& translator)
    : m_g(g),
      m_translator(translator),
      m_location_id()
  { m_location_id = m_translator(g->get_location_id()); }

  bool operator()(vertex_descriptor const& gid) const
  { return m_translator(m_g->locality(gid).location()) == m_location_id; }

  template<typename Edge>
  bool operator()(Edge const& e) const
  { return !this->operator()(e.target()); }

  void define_type(typer& t)
  {
    t.member(m_g);
    t.member(m_translator);
    t.member(m_location_id);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Predicate to translate a location to the hierarchy. For the
/// first level of hierarchy, this is identity.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct identity_translator
{
  location_type operator()(location_type const& loc) const
  { return loc; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Predicate to translate a location to the hierarchy.
///
/// Translates a physical location ID to its matching MPI process ID.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct translator
{
  location_type m_locs_per_process;

  translator(location_type locs_per_process)
    : m_locs_per_process(locs_per_process)
  { }

  location_type operator()(location_type const& loc) const
  { return loc / m_locs_per_process * m_locs_per_process; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Empty post-process functor when only a single level of
/// hierarchy is being created.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct empty_post_process
{
  template<typename Vertex, typename Comp>
  void operator()(Vertex, Comp const&) const
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Removes superedges from vertices in post-process for higher
/// levels of hierarchy.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct post_process
{
  template<typename Vertex, typename Comp>
  void operator()(Vertex v, Comp const& comp) const
  {
    for (auto e_it = v.begin(); e_it != v.end(); ++e_it) {
      if (comp(*e_it))
        v.property().remove_child_neighbor((*e_it).target());
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function enabling children inform their supervertex
/// about themselves (new child) and their adjacent edges and properties.
/// @tparam G The type of the graph container of the next level (same
/// type for all levels, but the lowest level).
/// @tparam GLL The type of the graph container of the lowest level
/// input graph.
/// @tparam LocalityFunc Identifies if an element is local to a given
/// hierarchical location/vertex.
/// @tparam TranslatorFunc The mapping of physical location to hierarchical
/// location/vertex.
/// @tparam PostProcess The functor to aid in deleting superedge metadata
/// from hierarchical vertices.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename G, typename GLL, typename LocalityFunc,
         typename TranslatorFunc, typename PostProcess>
struct children_inform_supervertex_wf
{
  typedef typename G::vertex_descriptor                vertex_desc_t;
  typedef typename G::edge_descriptor                  edge_desc_t;
  typedef sv_edge_property_add_child_wf<vertex_desc_t> e_wf_t;

  typedef add_child_edge_to_target_sv_wf<vertex_desc_t>
    add_child_edge_to_target_sv_wf_t;
  aggregator<add_child_edge_to_target_sv_wf_t, G> m_aggr;

  G*             m_g;
  GLL*           m_lower_level;
  location_type  m_location_id;
  bool           m_delete_edges;
  LocalityFunc   m_is_local;
  TranslatorFunc m_translator;
  PostProcess    m_post_process;

  location_type home_location(vertex_desc_t const& gid) const
  { return m_translator(m_lower_level->locality(gid).location()); }

  typedef void result_type;

  children_inform_supervertex_wf(G* g, GLL* lower_level,
                                 size_t max_msg_sz, bool delete_edges,
                                 LocalityFunc const& is_local,
                                 TranslatorFunc const& translator,
                                 PostProcess const& post_process = PostProcess()
                                 )
    : m_aggr(g, max_msg_sz), m_g(g), m_lower_level(lower_level),
      m_location_id(g->get_location_id()),
      m_delete_edges(delete_edges),
      m_is_local(is_local),
      m_translator(translator),
      m_post_process(post_process)
  { m_location_id = m_translator(m_location_id); }

  template<typename Vertex>
  void operator()(Vertex v)
  {
    const vertex_desc_t vd = v.descriptor();

    // Add/inform my supervertex's edge(s) of its children edges.
    for (auto const& e : v) {
      const vertex_desc_t target = e.target();
      const location_type target_home_location = home_location(target);
      if (!m_is_local(target)) {
        add_supervertex_edges(vd, target, target_home_location);
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add/inform my supervertex's edge(s) of its children edges.
  ///
  /// @param vd Descriptor of the child vertex (source of the child edge).
  /// @param target Descriptor of the child vertex's neighbor (target of
  /// the child edge).
  /// @param target_home_location home-location of the target vertex.
  /// @todo the aggregator used in this method should be changed to use the
  /// tunneling aggregator once it is available.
  //////////////////////////////////////////////////////////////////////
  void add_supervertex_edges(vertex_desc_t const& vd,
                             vertex_desc_t const& target,
                             location_type const& target_home_location)
  {
    // Add an edge to home-location of remote vertex.
    // NonMultiEdges Graph, so only one edge will be added and others
    // discarded. Also inform the edge of its children source-vertices.
    m_g->add_edge_async(edge_desc_t(m_location_id, target_home_location));
    m_g->distribution().vp_apply_async(m_location_id,
                                       e_wf_t(vd, target_home_location));

    /*****CHANGE THIS TO USE TUNNELING AGGREGATORS INSTEAD, WHEN READY*****/
    // Inform the target's supervertex of this child edge.
    m_aggr.add(add_child_edge_to_target_sv_wf_t(target_home_location,
                                                vd, target));
  }

  void define_type(typer& t)
  {
    t.member(m_aggr);
    t.member(m_g);
    t.member(m_lower_level);
    t.member(m_delete_edges);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper to create a level of hierarchy based on the machine
/// hierarchy, for use in @ref h_paradigm().
///
/// Creates a level of hierarchy based on the input @ref graph_view
/// and vertex-grouping by locality, i.e., all vertices on a location belong
/// to the same supervertex, and each location has one supervertex.
/// Supervertices and superedges in the resulting output graph store
/// descriptors of their children in the input.
/// @param gvw The input @ref graph_view over which the new level is to be
/// created.
/// @param locality_func Identifies if an element is local to a given
/// hierarchical location/vertex.
/// @param translator_func The mapping of physical location to hierarchical
/// location/vertex.
/// @param post_process_func The functor to aid in deleting superedge metadata
/// from hierarchical vertices.
/// @param max_msg_sz The number of requests to aggregate for the aggregator.
/// @param delete_edges Indicates if the non-local edges of the input
/// graph will be deleted. This is true by default, as the @ref h_paradigm
/// requires the edges to be deleted.
/// @return A graph_view over the output supergraph.
/// @note Destructive, the input graph view will be mutated to delete some
/// or all edges, but the information of the graph is maintained between
/// the input and the hierarchy.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GraphView, typename LocalityFunc, typename TranslatorFunc,
         typename PostProcessFunc>
graph_view<digraph<
  create_level_machine_detail::super_vertex_machine_property<GraphView> > >
create_level_machine_helper(GraphView& gvw, LocalityFunc const& locality_func,
                            TranslatorFunc const& translator_func,
                            PostProcessFunc const&,
                            bool delete_edges = true,
                            size_t max_msg_sz = 512)
{
  using namespace create_level_machine_detail;
  typedef typename GraphView::vertex_descriptor         vertex_desc_t;
  typedef super_vertex_machine_property<GraphView>      sv_property_t;
  typedef digraph<sv_property_t>                        graph_cont_t;
  typedef graph_view<graph_cont_t>                      graph_view_t;

  graph_cont_t* g = new graph_cont_t(gvw.get_num_locations(),
                                     sv_property_t(gvw));
  graph_view_t next_level_vw(g);

  gvw.sort_edges();


  typedef typename GraphView::view_container_type lower_level_cont_t;
  typedef children_inform_supervertex_wf<graph_cont_t, lower_level_cont_t,
                                         LocalityFunc, TranslatorFunc,
                                         PostProcessFunc> wf_t;
  // propagate children info. (ids and edges) to supervertex in next level graph
  // and set-up superedges:
  map_func(wf_t(g, gvw.get_container(), max_msg_sz, delete_edges,
                locality_func, translator_func), gvw);


  // If deleting edges is allowed, the edges of the current level
  // that span across partitions (supervertices) will be deleted,
  // so the current level is only left with intra-partition edges.
  if (delete_edges) {
    // Count the number of interpartition edges before we delete them
    map_func(
      add_cut_degree_wf<graph_cont_t, LocalityFunc>{g, locality_func}, gvw
    );

    // Delete duplicate and interpartition edges
    try_uncommit(gvw.container());

    gvw.container().remove_duplicate_edges();
    gvw.container().erase_edges_if(locality_func);

    try_commit(gvw.container());
  }


  return next_level_vw;
}

} // namespace create_level_machine_detail;


STAPL_PROXY_HEADER_TEMPLATE(
  create_level_machine_detail::super_vertex_machine_property, GraphView)
{
  STAPL_PROXY_DEFINES(
    create_level_machine_detail::super_vertex_machine_property<GraphView>)
  STAPL_PROXY_REFLECT_TYPE(vd_type)
  STAPL_PROXY_REFLECT_TYPE(child_cont_type)
  STAPL_PROXY_METHOD(add_child, vd_type)
  STAPL_PROXY_METHOD(add_child_neighbor, vd_type, vd_type)
  STAPL_PROXY_METHOD(remove_child_neighbor, vd_type)
  STAPL_PROXY_METHOD_RETURN(children_size, size_t)
  STAPL_PROXY_METHOD_RETURN(neighbors_size, size_t)
  STAPL_PROXY_METHOD_RETURN(cut_edges_size, std::size_t, vd_type)
  STAPL_PROXY_METHOD(add_cut_degree, vd_type, std::size_t)

  template<typename Func>
  inline void apply_updates_to_child_targets(vd_type source, Func const& f)
  {
    Accessor::invoke(&target_t::template apply_updates_to_child_targets<Func>,
                     source, f);
  }

  template<typename Func, typename View>
  inline void neighbors_apply(vd_type const& child_source, Func const& f,
                              View* vw) const
  {
    Accessor::invoke(&target_t::template neighbors_apply<Func, View>,
                     child_source, f, vw);
  }
}; //struct proxy



//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to create a level of hierarchy based on the machine
/// hierarchy, for use in @ref h_paradigm().
///
/// Creates a level of hierarchy based on the input @ref graph_view
/// and vertex-grouping by locality, i.e., all vertices on a location belong
/// to the same supervertex, and each location has one supervertex.
/// Supervertices and superedges in the resulting output graph store
/// descriptors of their children in the input.
/// @param gvw The input @ref graph_view over which the new level is to be
/// created.
/// @param max_msg_sz The number of requests to aggregate for the aggregator.
/// @param delete_edges Indicates if the non-local edges of the input
/// graph will be deleted. This is true by default, as the @ref h_paradigm
/// requires the edges to be deleted.
/// @return A graph_view over the output supergraph.
/// @note Destructive, the input graph view will be mutated to delete some
/// or all edges, but the information of the graph is maintained between
/// the input and the hierarchy.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GraphView>
graph_view<digraph<
  create_level_machine_detail::super_vertex_machine_property<GraphView> > >
create_level_machine(GraphView& gvw, bool delete_edges = true,
                     size_t max_msg_sz = 512)
{
  using namespace create_level_machine_detail;

  using h_view_type = graph_view<digraph<
    create_level_machine_detail::super_vertex_machine_property<GraphView>
  >>;

  typedef typename GraphView::view_container_type lower_level_cont_t;
  typedef identity_translator translate_func_t;
  typedef is_local_0<lower_level_cont_t, h_view_type> locality_func_t;

  return
    create_level_machine_helper(gvw,
                                locality_func_t(gvw.get_container()),
                                translate_func_t(), empty_post_process(),
                                delete_edges, max_msg_sz);
}


//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to create a level of hierarchy based on the machine
/// hierarchy, for use in @ref h2_paradigm().
///
/// Creates a level of hierarchy based on the input @ref graph_view
/// and vertex-grouping by locality, i.e., all vertices on an MPI process belong
/// to the same supervertex, and each MPI process has one supervertex.
/// Supervertices and superedges in the resulting output graph store
/// descriptors of their children in the input.
/// @param gvw The input @ref graph_view over which the new level is to be
/// created. This must already be a hierarchical view over locations, created by
/// calling @ref create_level_machine().
/// @param max_msg_sz The number of requests to aggregate for the aggregator.
/// @param delete_edges Indicates if the non-local edges of the input
/// graph will be deleted. This is true by default, as the @ref h_paradigm
/// requires the edges to be deleted.
/// @return A graph_view over the output supergraph.
/// @note Destructive, the input graph view will be mutated to delete some
/// or all edges, but the information of the graph is maintained between
/// the input and the hierarchy.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GraphView>
graph_view<digraph<
  create_level_machine_detail::super_vertex_machine_property<GraphView> > >
create_level2_machine(GraphView& gvw, bool delete_edges = true,
                      size_t max_msg_sz = 512)
{
  using namespace create_level_machine_detail;
  typedef typename GraphView::view_container_type lower_level_cont_t;
  typedef is_local_1<lower_level_cont_t, translator> locality_func_t;

  const size_t num_locs_per_process
    = runtime::this_context::get().get_gang_md().local_size();
  translator trans(num_locs_per_process);

  return
    create_level_machine_helper(gvw,
                                locality_func_t(gvw.get_container(), trans),
                                trans, post_process(),
                                delete_edges, max_msg_sz);
}

}  // namespace stapl

#endif

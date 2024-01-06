/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_CREATE_LEVEL_HUBS_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_CREATE_LEVEL_HUBS_HPP

#include <stapl/containers/graph/digraph.hpp>
#include <stapl/containers/graph/algorithms/aggregator.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/views/lazy_graph_view.hpp>
#include <stapl/domains/interval.hpp>
#include <boost/serialization/unordered_set.hpp>

namespace stapl {

namespace create_level_hubs_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex stub class for the @ref create_level_hubs algorithm.
///
/// Used as vertex property for the hubs supergraph and behaves like
/// a hub vertex. This provides functionality needed to support a
/// hierarchy of hubs.
///
/// @tparam VD The type of the vertex descriptor.
/// @tparam VP The type of the vertex property.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename VD, typename VP>
struct vertex_stub
{
  typedef VD  vertex_descriptor;
  typedef VP  property_type;
  typedef VP& property_reference;

  property_type m_property;
  bool          m_active;

  vertex_stub(void)
    : m_property(), m_active(false)
  { }

  property_reference property(void)
  { return m_property; }

  void reset(void)
  {
    m_property = property_type();
    m_active = false;
  }

  bool is_active(void) const
  { return m_active; }

  void set_active(bool b)
  { m_active = b; }

  void define_type(typer& t)
  { t.member(m_property); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property class for the @ref create_level_hubs algorithm.
/// @tparam GraphView The type of the lower-level input @ref graph_view.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GraphView>
struct super_vertex_hub_property
{
  typedef typename GraphView::vertex_descriptor vd_type;
  typedef typename GraphView::vertex_property   vp_type;

  typedef vertex_stub<vd_type, vp_type> vertex_type;

  /// Maps a source (child) vertex to its hubs.
  typedef boost::unordered_map<vd_type,
                               std::vector<vertex_type*> >  child_cont_type;

  /// @brief Maps a hub (child) descriptor to its reducer-vertex and a bool
  /// indicating if it was updated.
  typedef boost::unordered_map<vd_type, vertex_type>  hub_reducer_cont_type;

  using degrees_map_type = std::unordered_map<vd_type, std::size_t>;

  template<typename VPR>
  struct property_set_f
  {
    VPR m_vpr;
    vp_type m_property;

    property_set_f(VPR const& vpr, vp_type const& p)
      : m_vpr(vpr), m_property(p)
    { }

    template<typename V>
    void operator()(V&& v) const
    { m_vpr(v.property(), m_property); }

    void define_type(typer& t)
    {
      t.member(m_vpr);
      t.member(m_property);
    }
  };

  /// @brief Stores the vertex descriptors of children vertices,
  /// as well as their mapping to hub vertices.
  child_cont_type  m_children;

  /// @brief Stores the reduction of properties of hub vertices.
  /// These must be flushed-out to the actual hubs.
  hub_reducer_cont_type  m_hubs;

  /// Stores the user vertex property.
  GraphView*       m_lower_level;

  /// Map to store the number of cut edges for vertices in the lower graph
  degrees_map_type m_cut_degrees;

  super_vertex_hub_property(void)
    : m_lower_level(nullptr)
  { }

  super_vertex_hub_property(GraphView& g)
    : m_lower_level(&g)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the provided functor to reduced properties of hubs
  /// connected to the given source vertex.
  /// @param source The descriptor of the source vertex (from lower-level
  /// graph).
  /// @param f The functor to be applied to the hub targets.
  //////////////////////////////////////////////////////////////////////
  template<typename Func>
  void apply_updates_to_hubs(vd_type const& source, Func&& f)
  {
    auto it = m_children.find(source);
    if (it != m_children.end()) {
      for (auto const& hub_target : it->second) {
        f.template operator()<vertex_type&>(*hub_target);
        hub_target->set_active(true);
      }
    }
  }

  template<typename VPR>
  void flush_updates_to_hubs(VPR const& vpr)
  {
    for (auto& h : m_hubs) {
      if (h.second.is_active()) {
        m_lower_level->container().apply_set(h.first,
          property_set_f<VPR>(vpr, h.second.property()));
        h.second.reset();
      }
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a child vertex to this supervertex.
  /// @param source The descriptor of the source vertex (from lower graph).
  /// @param hub The descriptor of the hub vertex (from lower graph).
  //////////////////////////////////////////////////////////////////////
  void add_child(vd_type const& source, vd_type const& hub)
  { m_children[source].push_back(&m_hubs[hub]); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds the provided hub vertices to this supervertex.
  /// @param hubs Container specifying the descriptors of hub vertices
  /// (from lower graph).
  //////////////////////////////////////////////////////////////////////
  template<typename HubsIDCont>
  void hubs(HubsIDCont const& hubs_cont)
  {
    for (auto const& h : hubs_cont)
      m_hubs.emplace(h, vertex_type());
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
    t.member(m_children);
    t.member(m_lower_level);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to initialize hub-vertices.
/// @tparam HubCont The type of the container of hub-descriptors.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename HubCont>
struct init_sv_hubs
{
  HubCont m_hubs;

  init_sv_hubs(HubCont const& hubs)
    : m_hubs(hubs)
  { }

  typedef void result_type;

  template<typename Vertex>
  result_type operator()(Vertex v) const
  { v.property().hubs(m_hubs); }

  void define_type(typer& t)
  { t.member(m_hubs); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to find hub-vertices.
///
/// Selects all vertices with out-degree greater than or equal to a
/// given threshold as hub vertices.
/// @tparam VD The type of the vertex descriptors.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename VD>
struct find_hubs_wf
{
  size_t m_k;

  find_hubs_wf(size_t k)
    : m_k(k)
  { }

  typedef boost::unordered_set<VD> result_type;

  template<typename Vertex>
  result_type operator()(Vertex v) const
  {
    result_type r;
    if (v.size() >= m_k)
      r.insert(v.descriptor());
    return r;
  }

  void define_type(typer& t)
  { t.member(m_k); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reduce functor to find hub-vertices.
///
/// Selects all vertices with out-degree greater than a given threshold
/// as hub vertices.
/// @tparam VD The type of the vertex descriptors.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename VD>
struct reduce_hubs_wf
{
  typedef boost::unordered_set<VD> result_type;

  template<typename T>
  result_type operator()(T const& t1, T const& t2) const
  {
    result_type r(t1);
    result_type temp(t2);
    r.insert(temp.begin(), temp.end());
    return r;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to add a child source vertex and its target to
/// a supervertex.
/// @tparam VD Type of the vertex-descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VD>
struct add_child_to_sv_wf
{
  typedef boost::unordered_set<VD> hubs_id_t;
  VD m_child_source;
  hubs_id_t m_hub_targets;

  add_child_to_sv_wf(VD const& child_source, hubs_id_t& hub_targets)
    : m_child_source(child_source), m_hub_targets(hub_targets)
  { }

  typedef void result_type;

  template<typename SuperVertexProperty>
  result_type operator() (SuperVertexProperty&& svp) const
  {
    for (auto const& h : m_hub_targets)
      svp.add_child(m_child_source, h);
  }

  void define_type(typer& t)
  {
    t.member(m_child_source);
    t.member(m_hub_targets);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Predicate that returns true if an edge's target is to a hub
//////////////////////////////////////////////////////////////////////
template<typename Hubs>
struct target_is_hub
{
  /// IDs of hubs
  Hubs m_hubs;

  target_is_hub(Hubs hubs)
    : m_hubs(std::move(hubs))
  { }

  template<class Edge>
  bool operator()(Edge const& e)
  {
    return m_hubs.count(e.target());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function enabling children inform their supervertex
/// about themselves (new child) and their adjacent edges and properties.
/// @tparam G The type of the graph container of the next level (same
/// type for all levels).
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename G, typename LowerView>
struct children_inform_supervertex_wf
{
  typedef typename LowerView::vertex_descriptor                  vertex_desc_t;
  typedef boost::unordered_set<vertex_desc_t>              hubs_id_t;

  hubs_id_t     m_hubs;
  G*            m_g;
  location_type m_location_id;
  bool          m_delete_edges;

  struct comp_target
  {
    template <class Edge1, class Edge2>
    bool operator()(Edge1 const& e1, Edge2 const& e2) const
    { return e1.target() < e2.target(); }
  };

  struct same_target
  {
    template <class Edge1, class Edge2>
    bool operator()(Edge1 const& e1, Edge2 const& e2) const
    { return e1.target() == e2.target(); }
  };


  typedef void result_type;

  children_inform_supervertex_wf(hubs_id_t const& hubs,
                                 G* g, bool delete_edges)
    : m_hubs(hubs), m_g(g), m_location_id(g->get_location_id()),
      m_delete_edges(delete_edges)
  { }

  template<typename Vertex>
  result_type operator()(Vertex v)
  {
    const vertex_desc_t vd = v.descriptor();

    hubs_id_t target_hubs;
    for (auto const& e : v)
      if (m_hubs.count(e.target()))
        target_hubs.insert(e.target());

    m_g->vp_apply_async(m_location_id,
                        add_child_to_sv_wf<vertex_desc_t>(vd, target_hubs));
  }

  void define_type(typer& t)
  {
    t.member(m_hubs);
    t.member(m_g);
    t.member(m_delete_edges);
  }
};


struct second_select
{
  template<typename H>
  bool operator()(H const& h1, H const& h2) const
  { return h1.second < h2.second; }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work-function to store in a supervertex the cut degree for
///        each vertex in the lower graph.
/// @tparam HubsGraph The hierarchical hub graph.
/// @tparam Hubs Type of the container of hubs (unordered_set)
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename HubsGraph, typename Hubs>
struct add_cut_degree_wf
{
  mutable HubsGraph* m_h_graph;
  Hubs m_hubs;
  location_type m_loc;

  add_cut_degree_wf(HubsGraph* h_graph, Hubs hubs)
    : m_h_graph(h_graph), m_hubs(std::move(hubs)),
      m_loc(h_graph->get_location_id())
  { }

  using result_type = void;

  template<typename V>
  result_type operator()(V&& v) const
  {
    using edge_type = typename std::decay<V>::type::adj_edges_type::value_type;

    std::size_t cut_degree =
      std::count_if(v.begin(), v.end(), [this](edge_type const& e) {
        return this->m_hubs.count(e.target());
      });

    m_h_graph->operator[](m_loc).property().add_cut_degree(
      v.descriptor(), cut_degree
    );
  }

  void define_type(typer& t)
  {
    t.member(m_h_graph);
    t.member(m_hubs);
    t.member(m_loc);
  }
};

} // namespace create_level_hubs_detail;


STAPL_PROXY_HEADER_TEMPLATE(
  create_level_hubs_detail::super_vertex_hub_property, GraphView)
{
  STAPL_PROXY_DEFINES(
    create_level_hubs_detail::super_vertex_hub_property<GraphView>)
  STAPL_PROXY_REFLECT_TYPE(vd_type)
  STAPL_PROXY_REFLECT_TYPE(child_cont_type)
  STAPL_PROXY_REFLECT_TYPE(hub_reducer_cont_type)
  STAPL_PROXY_METHOD(add_child, vd_type, vd_type)
  STAPL_PROXY_METHOD_RETURN(cut_edges_size, std::size_t, vd_type)
  STAPL_PROXY_METHOD(add_cut_degree, vd_type, std::size_t)

  template<typename Func>
  void apply_updates_to_hubs(vd_type source, Func const& f)
  {
    Accessor::invoke(&target_t::template apply_updates_to_hubs<Func>,
                     source, f);
  }

  template<typename VPR>
  void flush_updates_to_hubs(VPR const& vpr)
  {
    Accessor::invoke(&target_t::template flush_updates_to_hubs<VPR>,
                     vpr);
  }

  template<typename HubsIDCont>
  void hubs(HubsIDCont const& hubs_cont)
  {
    Accessor::invoke(&target_t::template hubs<HubsIDCont>, hubs_cont);
  }

}; //struct proxy



//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to create a level of hierarchy based on hubs,
/// for use in @ref h_hubs_paradigm().
///
/// Creates a level of hierarchy based on the input @ref graph_view
/// and high-degree vertices (hubs) in the input graph, i.e., all hub
/// vertices will have hub-vertex representative supervertices on each
/// location.
/// Supervertices and superedges in the resulting output graph store
/// descriptors of their children in the input.
///
/// For use with the @ref h_hubs_paradigm(), this must be called before
/// calling @ref create_level_machine() on the input graph.
///
/// The algorithm first finds all vertices qualifying as hubs (i.e. have
/// out-degree greater than the given threshold (@p k). A new level of
/// hierarchy is created with one supervertex per location, and each such
/// supervertex contains representatives of all hub vertices. These
/// representatives can only be written to, and their values can be
/// flushed to the actual hub-vertices in the input graph.
///
/// This incurs O(h) storage per location, where h is the number of hubs,
/// as defined by the hub cutoff @p k.
///
/// @param gvw The input @ref graph_view over which the new level is to be
/// created.
/// @param k The cutoff degree for considering a vertex as a hub. All vertices
/// with out-degree greater than or equal to k are considered hub vertices.
/// @param delete_edges Indicates if the non-local edges for the input
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
  create_level_hubs_detail::super_vertex_hub_property<GraphView> > >
create_level_hubs(GraphView& gvw, size_t k, bool delete_edges = true)
{
  using namespace create_level_hubs_detail;
  typedef typename GraphView::vertex_descriptor         vertex_desc_t;
  typedef super_vertex_hub_property<GraphView>          sv_property_t;
  typedef digraph<sv_property_t>                        graph_cont_t;
  typedef graph_view<graph_cont_t>                      graph_view_t;

  typedef boost::unordered_set<vertex_desc_t> hubs_cont_t;

  hubs_cont_t hubs = map_reduce(find_hubs_wf<vertex_desc_t>(k),
                                reduce_hubs_wf<vertex_desc_t>(),
                                gvw);

  graph_cont_t* g = new graph_cont_t(gvw.get_num_locations(),
                                     sv_property_t(gvw));
  graph_view_t next_level_vw(g);

  map_func(init_sv_hubs<hubs_cont_t>(hubs), next_level_vw);


  typedef children_inform_supervertex_wf<graph_cont_t, GraphView> wf_t;
  // propagate children info. (ids and edges) to supervertex in
  // next level graph.
  map_func(wf_t(hubs, g, delete_edges), gvw);

  // Delete edges whose targets are hubs
  if (delete_edges) {
    map_func(add_cut_degree_wf<graph_cont_t, hubs_cont_t>{g, hubs}, gvw);

    try_uncommit(gvw.container());
    gvw.container().remove_duplicate_edges();
    gvw.container().erase_edges_if(target_is_hub<hubs_cont_t>{hubs});
    try_commit(gvw.container());
  }

  gvw.sort_edges();

  return next_level_vw;
}

}  // namespace stapl

#endif

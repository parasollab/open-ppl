/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_CREATE_LEVEL_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_CREATE_LEVEL_HPP

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/aggregator.hpp>
#include <stapl/views/proxy_macros.hpp>

#include <stapl/containers/array/static_array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/graph/algorithms/nc_map.hpp>

namespace stapl {


//////////////////////////////////////////////////////////////////////
/// @brief Vertex property class for the create_level algorithm.
/// @tparam Property The type of the user-property.
/// @ingroup pgraphAlgo
///
/// This property wraps around the user-provided vertex property and
/// provides functionality needed to support a hierarchy of levels.
/// Any graph calling the create_level algorithm must use this as its
/// vertex property.
//////////////////////////////////////////////////////////////////////
template<typename Property>
struct super_vertex_property
{
  typedef Property property_type;
  typedef std::vector<size_t> child_cont_type;

  /// Stores the vertex descriptors of children vertices.
  std::vector<size_t>  children;
  /// Stores the user vertex property.
  property_type        property;
  /// ID of the super-vertex of this vertex, if any.
  size_t               m_supervertex_id;

  super_vertex_property()
    : children(), property(),
      m_supervertex_id(std::numeric_limits<size_t>::max())
  { }

   super_vertex_property(Property const& p)
     : children(), property(p),
       m_supervertex_id(std::numeric_limits<size_t>::max())
   { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the provided functor to children vertices of this vertex
  /// belonging to the provided graph,
  /// @param g The graph_view of the graph at the level below. This graph
  /// owns the children vertices.
  /// @param f The functor to be applied to the children.
  //////////////////////////////////////////////////////////////////////
  template<typename GraphView, typename Func>
  void children_apply(GraphView g, Func const& f)
  {
    for (child_cont_type::const_iterator it = children.begin();
         it != children.end(); ++it)
      g.vp_apply_async(*it, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a child vertex to this supervertex.
  /// @param i The descriptor of the child vertex of this supervertex.
  //////////////////////////////////////////////////////////////////////
  void add_child(size_t i)
  { children.push_back(i); }

  size_t supervertex() const
  { return m_supervertex_id; }

  void supervertex(size_t i)
  { m_supervertex_id = i; }

  void define_type(typer& t)
  {
    t.member(children);
    t.member(property);
    t.member(m_supervertex_id);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for the @ref super_vertex_property.
/// @tparam Property The user-property for the vertex.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Property, class Accessor>
class proxy<super_vertex_property<Property>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef super_vertex_property<Property> target_t;

public:

  typedef typename target_t::property_type property_type;
  typedef typename target_t::child_cont_type child_cont_type;

  STAPL_PROXY_MEMBER(property, property_type);
  STAPL_PROXY_MEMBER(children, child_cont_type);

  inline explicit proxy(Accessor const& acc)
    : Accessor(acc), property(acc), children(acc)
  { }

  inline operator target_t() const
  { return Accessor::read(); }

  inline proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  inline proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}


  template<typename GraphView, typename Func>
  inline void children_apply(GraphView& g, Func const& f)
  {
    Accessor::invoke(&target_t::template children_apply<GraphView,Func>,
                     g, f);
  }

  void add_child(size_t i)
  { Accessor::invoke(&target_t::add_child, i); }

  size_t supervertex() const
  {
    size_t (target_t::*pmf)(void) const = &target_t::supervertex;
    return Accessor::const_invoke(pmf);
  }

  inline void supervertex(size_t i)
  { Accessor::invoke(&target_t::supervertex, i); }

}; //struct proxy


//////////////////////////////////////////////////////////////////////
/// @brief Edge property class for the create_level algorithm.
/// @tparam Property The type of the user-property.
/// @ingroup pgraphAlgo
///
/// This property wraps around the user-provided edge property and
/// provides functionality needed to support a hierarchy of levels.
/// Any graph calling the create_level algorithm must use this as its
/// edge property.
//////////////////////////////////////////////////////////////////////
template<typename Property>
struct super_edge_property
{
  typedef size_t VD;
  typedef edge_descriptor_impl<VD> ED;
  typedef Property property_type;
  typedef std::vector<ED> child_cont_type;

  /// Stores the edge descriptors of children edges.
  child_cont_type      children;
  /// Stores the user edge property.
  property_type        property;
  /// ID of the super-vertex of the target-vertex of this edge, if any.
  VD                   m_supervertex_id;

  super_edge_property()
    : children(),
      property(),
      m_supervertex_id(std::numeric_limits<VD>::max())
  { }

   super_edge_property(Property const& p)
     : children(),
       property(p),
       m_supervertex_id(std::numeric_limits<VD>::max())
   { }

  super_edge_property(Property const& p, ED const& e)
     : children(),
       property(p),
       m_supervertex_id(std::numeric_limits<VD>::max())
  { children.push_back(e); }

  super_edge_property(Property const& p, child_cont_type const& ch)
     : children(ch),
       property(p),
       m_supervertex_id(std::numeric_limits<VD>::max())
   { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the provided functor to children edges of this edge
  /// belonging to the provided graph,
  /// @param g The graph_view of the graph at the level below. This graph
  /// owns the children edges.
  /// @param f The functor to be applied to the children.
  //////////////////////////////////////////////////////////////////////
  template<typename GraphView, typename Func>
  void children_apply(GraphView& g, Func const& f) const
  {
    for (child_cont_type::const_iterator it = children.begin();
         it != children.end(); ++it)
      g.ep_apply_async(*it, f);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a child edge to this superedge.
  /// @param i The descriptor of the child edge of this superedge.
  //////////////////////////////////////////////////////////////////////
  void add_child(ED const& i)
  { children.push_back(i); }

  VD target_supervertex() const
  { return m_supervertex_id; }

  void target_supervertex(VD const& i)
  { m_supervertex_id = i; }

  void define_type(typer& t)
  {
    t.member(children);
    t.member(property);
    t.member(m_supervertex_id);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for the @ref super_edge_property.
/// @tparam Property The user-property for the edge.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <class Property, class Accessor>
class proxy<super_edge_property<Property>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef super_edge_property<Property> target_t;

public:
  typedef typename target_t::VD VD;
  typedef typename target_t::ED ED;
  typedef typename target_t::property_type property_type;
  typedef typename target_t::child_cont_type child_cont_type;

  STAPL_PROXY_MEMBER(property, property_type);
  STAPL_PROXY_MEMBER(children, child_cont_type);

  inline explicit proxy(Accessor const& acc)
    : Accessor(acc), property(acc), children(acc)
  { }

  inline operator target_t() const
  { return Accessor::read(); }

  inline proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  inline proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}


  template<typename GraphView, typename Func>
  inline void children_apply(GraphView& g, Func const& f) const
  {
    Accessor::const_invoke(&target_t::template children_apply<GraphView,Func>,
                           g, f);
  }

  inline void add_child(ED const& i)
  { Accessor::invoke(&target_t::add_child, i); }

  VD target_supervertex() const
  {
    VD (target_t::*pmf)(void) const = &target_t::target_supervertex;
    return Accessor::const_invoke(pmf);
  }

  inline void target_supervertex(VD const& i)
  { Accessor::invoke(&target_t::target_supervertex, i); }

}; //struct proxy


namespace create_level_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Identifies super-vertex positions in an array by setting
/// the array-element corresponding to the leader-vertex as a one (1)
/// and other elements as a zero (0). Leader vertices are ones that
/// are matched to themselves as specified in the vertex property map.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct set_leaders_wf
{
  typedef void result_type;

  template<typename Vertex, typename ArrayElt, typename PropMap>
  void operator()(Vertex v, ArrayElt e, PropMap& gm)
  {
    if (gm.get(v) == v.descriptor())  // I am leader.
      e = 1;
    else
      e = 0;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Sets the supervertex-IDs of leader vertices to the descriptor
/// specified by the corrsponding array element. Leader vertices are
/// ones that are matched to themselves as specified in the vertex
/// property map.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct set_svid_for_leaders
{
  typedef void result_type;

  template<typename Vertex, typename ArrayElt, typename PropMap>
  void operator()(Vertex v, ArrayElt e, PropMap& gm)
  {
    if (gm.get(v) == v.descriptor())  // I am leader.
      v.property().supervertex(e);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Sets the supervertex-IDs of vertices to the specified ID.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct set_supervertex_id
{
  size_t m_id;

  typedef void result_type;

  set_supervertex_id(size_t id)
    : m_id(id)
  { }

  template<typename Property>
  void operator()(Property& p) const
  { p.supervertex(m_id); }

  void define_type(typer& t)
  { t.member(m_id); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns the supervertex-IDs of leader vertices.
/// @tparam G The type of graph pContainer.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename G>
struct get_svid
{
  typedef size_t result_type;

  template<typename Property>
  size_t operator()(Property& p) const
  {
    return p.supervertex();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Sets the supervertex ID of vertices in the graphs to be the
/// same as the supervertex ID from the corresponding leader vertex.
/// @tparam G The type of graph pContainer.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename G>
struct fetch_svid_from_leaders
{
  G* m_g;

  typedef void result_type;

  fetch_svid_from_leaders(G* g)
    : m_g(g)
  { }

  template<typename Vertex, typename ArrayElt, typename PropMap>
  void operator()(Vertex v, ArrayElt e, PropMap& gm)
  {
    size_t group_leader_id = gm.get(v);
    if (group_leader_id != v.descriptor()) // I am not leader.
      v.property().supervertex(m_g->vp_apply(group_leader_id, get_svid<G>()));
  }

  void define_type(typer& t)
  { t.member(m_g); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Sets the supervertex ID of the target vertex of the edge.
/// This is useful in finding which supervertex the target belongs to
/// just by locally reading the outgoing edge.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct set_supervertex_edge_wf
{
  size_t m_id;

  typedef void result_type;

  set_supervertex_edge_wf(size_t id)
    : m_id(id)
  { }

  template<typename Property>
  void operator()(Property& p) const
  { p.target_supervertex(m_id); }

  void define_type(typer& t)
  { t.member(m_id); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper functor for using aggregators to apply functors to
/// edge properties.
/// @tparam ED The type of the edge descriptor.
/// @tparam WF The type of functor to apply to the edge.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename ED, typename WF>
struct ep_apply_aggr_wf
{
  ED m_eid;
  WF m_wf;

  typedef void result_type;

  ep_apply_aggr_wf(ED const& eid, WF const& wf)
    : m_eid(eid), m_wf(wf)
  { }

  size_t target() const
  { return m_eid.source(); }

  template<typename Graph>
  void operator()(Graph* g) const
  { g->ep_apply_async(m_eid, m_wf); }

  void define_type(typer& t)
  {
    t.member(m_eid);
    t.member(m_wf);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Marks the incoming adjacent edges of a vertex with its
/// supervertex ID. The incoming adjacent in an undirected graph is
/// the reverse of the outgoing adjacent edge, i.e. (target, source).
/// This is useful in finding which supervertex the target belongs to
/// just by locally reading the outgoing edge.
/// @tparam G The type of graph pContainer.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename G>
struct mark_edge_with_target_svid
{
  typedef typename G::edge_descriptor edge_desc_t;
  typedef ep_apply_aggr_wf<edge_desc_t,
                           set_supervertex_edge_wf> ep_apply_aggr_wf_t;
  aggregator_apply<ep_apply_aggr_wf_t, G,
                   aggr_algo_detail::aggregator_helper_wf> m_aggr;

  typedef void result_type;

  mark_edge_with_target_svid(G* g, size_t const& max_msg_sz)
    : m_aggr(g, max_msg_sz)
  { }

  template<typename Vertex>
  void operator()(Vertex v)
  {
    set_supervertex_edge_wf sv(v.property().supervertex());
    for (auto const& e : v)
      m_aggr.add(ep_apply_aggr_wf_t(edge_desc_t(e.target(), e.source()), sv));
  }

  void define_type(typer& t)
  { t.member(m_aggr); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to add children vertex/edge descriptors to
/// parent supervertex/superedge.
/// @tparam P The type of the user vertex/edge property.
/// @tparam PReducer The user-defined reducer for vertex/edge-properties.
/// @tparam Descriptor The type of the vertex/edge descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename P, typename PReducer, typename Descriptor>
struct set_children_wf
{
  Descriptor m_id;
  P          m_prop;

  typedef void result_type;

  set_children_wf(Descriptor const& id, P const& prop)
    : m_id(id), m_prop(prop)
  { }

  template<typename Property>
  void operator()(Property& p) const
  {
    p.add_child(m_id);
    p.property = PReducer()(p.property, m_prop);
  }

  void define_type(typer& t)
  {
    t.member(m_id);
    t.member(m_prop);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to set-up supervertex and its edges in one-shot.
/// @tparam VertexWF The user-defined reducer for vertex-properties.
/// @tparam EdgeWF The user-defined reducer for edge-properties.
/// @tparam ED The type of the edge descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename VertexWF, typename EdgeWF, typename ED>
struct super_vertex_aggregate_setup_functor
{
  typedef std::vector<std::pair<size_t, EdgeWF> > e_wf_cont_t;

  size_t       m_super_vertex_id;
  VertexWF     m_v_wf;
  e_wf_cont_t  m_e_wf_vec;

  typedef void result_type;

  super_vertex_aggregate_setup_functor(size_t const& supervertex,
                                       VertexWF const& v_wf)
    : m_super_vertex_id(supervertex), m_v_wf(v_wf), m_e_wf_vec()
  { }

  size_t target() const
  { return m_super_vertex_id; }

  void add_edge(size_t const& target_supervertex, EdgeWF const& e_wf)
  { m_e_wf_vec.push_back(std::make_pair(target_supervertex, e_wf)); }

  template<typename Graph>
  void operator() (Graph* g) const
  {
    typedef typename Graph::edge_property eprop_t;
    g->vp_apply_async(m_super_vertex_id, m_v_wf);
    for (typename e_wf_cont_t::const_iterator cit = m_e_wf_vec.begin();
         cit != m_e_wf_vec.end(); ++cit) {
      ED ed(m_super_vertex_id, cit->first);
      if (!g->ep_find_apply(ed, cit->second))
        g->add_edge_async(ed,
                          eprop_t(cit->second.m_prop,
                                  cit->second.m_id));
    }
  }

  void define_type(typer& t)
  {
    t.member(m_super_vertex_id);
    t.member(m_v_wf);
    t.member(m_e_wf_vec);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function enabling children inform their supervertex
/// about themselves (new child) and their adjacent edges and properties.
/// @tparam G The type of the graph container of the next level (same
/// type for all levels).
/// @tparam VPReducer The user-defined reducer for vertex-properties.
/// @tparam EPReducer The user-defined reducer for edge-properties.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename G, typename VPReducer, typename EPReducer>
struct children_inform_supervertex_wf
{
  typedef typename G::vertex_descriptor vertex_desc_t;
  typedef typename G::edge_descriptor   edge_desc_t;

  typedef set_children_wf<typename G::vertex_property::property_type,
                          VPReducer, vertex_desc_t> v_wf_t;

  typedef set_children_wf<typename G::edge_property::property_type,
                          EPReducer, edge_desc_t> e_wf_t;

  typedef super_vertex_aggregate_setup_functor<v_wf_t, e_wf_t, edge_desc_t>
    sv_aggr_setup_wf_t;

  /// Indicates whether or not to add self-edges in supervertex.
  bool m_self_edges;
  aggregator_apply<sv_aggr_setup_wf_t, G,
                   aggr_algo_detail::aggregator_helper_wf> m_aggr;

  typedef void result_type;

  children_inform_supervertex_wf(G* g, size_t max_msg_sz, bool self_edges=false)
    : m_self_edges(self_edges), m_aggr(g, max_msg_sz)
  { }

  template<typename Vertex>
  void operator()(Vertex v)
  {
    // inform my supervertex that I'm a child.
    sv_aggr_setup_wf_t x(v.property().supervertex(),
                         v_wf_t(v.descriptor(), v.property().property));

    // Add/inform my supervertex's edge(s) of its children edges.
    for (typename Vertex::adj_edge_iterator aei = v.begin();
         aei != v.end(); ++aei) {

      // no self-edges.
      if (m_self_edges ||
          v.property().supervertex() != (*aei).property().target_supervertex())
      {
        // NME Graph, so only one edge will be added and others discarded.
        // Also inform the edge of its children edges and reduce property.
        x.add_edge((*aei).property().target_supervertex(),
                   e_wf_t((*aei).descriptor(), (*aei).property().property));
      }
    }
    m_aggr.add(x);
  }

  void define_type(typer& t)
  {
    t.member(m_self_edges);
    t.member(m_aggr);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Comparator for comparing targets of two edges.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct comp
{
  typedef bool result_type;

  template <typename Edge1, typename Edge2>
  bool operator()(Edge1 const& e1, Edge2 const& e2) const
  { return e1.target() < e2.target(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to sort all adjacents of a vertex.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct set_up_supervertex_wf
{
  typedef void result_type;

  template<typename Vertex>
  void operator()(Vertex v)
  { sort(v.begin(), v.end(), comp()); }
};

} // namespace create_level_detail;



//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to create an entire level of hierarchy at once.
/// @ingroup pgraphAlgo
///
/// Creates a new level of hierarchy based on the input @ref graph_view
/// and the provided vertex-grouping. Supervertices and superedges in the
/// resulting output graph store descriptors of their children in the input.
/// Properties of supervertices and superedges are reductions of the properties
/// of their children through the user provided reducers.
/// @param gvw The input @ref graph_view over which the new level is to be
/// created. The underlying graph container must be DIRECTED, NONMULTIEDGES,
/// and store the custom @ref super_vertex_property and
/// @ref super_edge_property on their vertices and edges, respectively.
/// @param vertex_group_map A vertex property map identifying the "group" of
/// each vertex in the graph. The leader-vertex of each group is the
/// vertex whose group ID is the same as its descriptor. There must be
/// exactly one leader vertex in each group. All non-leader vertices must
/// indicate one of the leader-vertices as their group, with whom they
/// would like to "collapse" to form the supervertex.
/// [vertex->vertex_descriptor].
/// @param vpr The vertex property reducer for reducing child-vertex
/// properties to form the supervertex property.
/// @param epr The edge property reducer for reducing child-edge
/// properties to form the superedge property.
/// @param max_msg_sz The number of requests to aggregate for the aggregator.
/// @param self_edges Indicates whether self-edges are allowed on the output
/// graph or not. If the supergraph has self-edges, they will represent the
/// internal edges between the supervertex's child vertices.
/// @param sort_edges Indicates if the adjacent-edges for each supervertex
/// in the output supergraph will be sorted based on their target-descriptors.
/// @return A graph_view over the output supergraph. The type of this view
/// is the same as the input graph_view.
//////////////////////////////////////////////////////////////////////
template<typename GraphView, typename PropMap,
         typename VertexPReducer, typename EdgePReducer>
GraphView create_level(GraphView& gvw, PropMap& vertex_group_map,
                       VertexPReducer const&, EdgePReducer const&,
                       size_t max_msg_sz = 512,
                       bool self_edges = false, bool sort_edges = false)
{
  using namespace create_level_detail;
  typedef typename GraphView::view_container_type graph_cont_t;
  size_t level_size = 0;

  {
    static_array<size_t> leader_array(gvw.size());
    array_view<static_array<size_t> > avw(leader_array);
    nc_map_func(set_leaders_wf(), gvw, avw, make_repeat_view(vertex_group_map));

    level_size = nc_partial_sum_accumulate(avw, avw, 0, true);

    nc_map_func(set_svid_for_leaders(), gvw, avw,
                make_repeat_view(vertex_group_map));

    nc_map_func(fetch_svid_from_leaders<graph_cont_t>(gvw.get_container()),
                gvw, avw, make_repeat_view(vertex_group_map));
    // Now everyone should have had their super-vertex-ids setup.
  }

  nc_map_func(mark_edge_with_target_svid<graph_cont_t>(gvw.get_container(),
                                                       max_msg_sz), gvw);

  graph_cont_t* g = new graph_cont_t(level_size);
  GraphView next_level_vw(g);

  // propagate children info. (ids and properties)
  // to supervertex in next level graph
  // and set-up superedges:
  nc_map_func(children_inform_supervertex_wf<graph_cont_t,
              VertexPReducer, EdgePReducer>(g, max_msg_sz, self_edges),
              gvw);

  if (sort_edges) {
    // set-up the super-vertices, sort the edges:
    nc_map_func(set_up_supervertex_wf(), next_level_vw);
  }

  return next_level_vw;
}

//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to create an entire level of hierarchy at once.
/// @ingroup pgraphAlgo
///
/// Creates a new level of hierarchy based on the input @ref graph_view
/// and the provided vertex-grouping. Supervertices and superedges in the
/// resulting output graph store descriptors of their children in the input.
/// Properties of supervertices and superedges are reductions of the properties
/// of their children through the user provided reducers.
/// @param gvw The input @ref graph_view over which the new level is to be
/// created. The underlying graph container must be DIRECTED, NONMULTIEDGES,
/// and store the custom @ref super_vertex_property and
/// @ref super_edge_property on their vertices and edges, respectively.
/// @param vertex_grouping An std::pair of a bool and a vertex property map.
/// The bool indicates whether the matching information is complete (true)
/// or partial (false). The vertex property map identifies the "group" of
/// each vertex in the graph. The leader-vertex of each group is the
/// vertex whose group ID is the same as its descriptor. There must be
/// exactly one leader vertex in each group.
/// All non-leader vertices must either chose indicate one of the
/// leader-vertices as their group, with whom they would like to "collapse"
/// to form the supervertex (complete grouping), OR the non-leader vertices
///  may choose to "collapse" with other leader or non-leader vertices
/// (partial grouping). In case of partial grouping, the final grouping
/// with which to create the supervertex is calculated by running a
/// pointer-jumping (or similar) algorithm . [vertex->vertex_descriptor]
/// @param vpr The vertex property reducer for reducing child-vertex
/// properties to form the supervertex property.
/// @param epr The edge property reducer for reducing child-edge
/// properties to form the superedge property.
/// @param max_msg_sz The number of requests to aggregate for the aggregator.
/// @param self_edges Indicates whether self-edges are allowed on the output
/// graph or not. If the supergraph has self-edges, they will represent the
/// internal edges between the supervertex's child vertices.
/// @param sort_edges Indicates if the adjacent-edges for each supervertex
/// in the output supergraph will be sorted based on their target-descriptors.
/// @return A graph_view over the output supergraph. The type of this view
/// is the same as the input graph_view.
//////////////////////////////////////////////////////////////////////
template<typename GraphView, typename PropMap,
         typename VertexPReducer, typename EdgePReducer>
GraphView create_level(GraphView& gvw,
                       std::pair<bool, PropMap> const& vertex_grouping,
                       VertexPReducer vpr, EdgePReducer epr,
                       size_t max_msg_sz = 512,
                       bool self_edges = false, bool sort_edges = false)
{
  if (vertex_grouping.first)
    return create_level(gvw, vertex_grouping.second, vpr, epr,
                        max_msg_sz, self_edges, sort_edges);
  else
    return create_level_partial_info(gvw, vertex_grouping.second, vpr, epr,
                                     max_msg_sz, self_edges);
}

}  // namespace stapl

#endif

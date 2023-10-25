/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PSCC_UTILS_H
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PSCC_UTILS_H

#include <stapl/containers/type_traits/index_bounds.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/repeated_view.hpp>
#include <set>

namespace stapl {

namespace algo_details {

//////////////////////////////////////////////////////////////////////
/// @brief Counts the number of uncolored nodes in a graph. It is an
/// important stopping condition for many of the pscc algorithms.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct count_size
{
  typedef size_t    result_type;

  template<typename Vertex>
  result_type operator()(Vertex v)
  {
    v.property().clear_marks();
    return !v.property().has_valid_cc();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A wrapper for the @ref count_size functor.
/// @param gv A @ref graph_view.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename GraphView>
size_t graph_free_node_count(GraphView gv)
{
  return map_reduce(count_size(), plus<size_t>(), gv);
}


//////////////////////////////////////////////////////////////////////
/// @brief Adds a copy of vertex v to graph g.
/// Used for copying the input graph to pscc's temp graph.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct add_verts_wf
{
  typedef void result_type;

  template <typename VRef, typename GView>
  void operator()(VRef v, GView g) const
  {
    typedef typename GView::vertex_property   prop;
    g.add_vertex(v.descriptor(), prop());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Adds a reverse edge to the property of each target node.
/// Used for copying the input graph to pscc's temp graph.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertexDescriptor>
struct add_predecessor
{
  typedef void      result_type;

  VertexDescriptor m_v;
  add_predecessor(VertexDescriptor const& v)
    : m_v(v)
  { }

  template <typename Property>
  void operator()(Property& p) const
  { p.add_predecessor(m_v); }

  void define_type(typer& t)
  { t.member(m_v); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Copies the edges of vertex v to graph g, adding both forward
/// and reverse copies of the edge.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct add_all_edges_wf
{
  typedef void result_type;

  template <typename VRef, typename GView>
  void operator()(VRef v, GView g) const
  {
    typedef typename GView::vertex_descriptor   vertex_type;
    typedef add_predecessor<vertex_type>        add_pred;

    for (auto&& i : v) {
      g.add_edge_async(i.descriptor());
      g.vp_apply_async(i.target(), add_pred(v.descriptor()));
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Creates a copy of the source graph @p s to the destination
/// graph @p d.
/// The vertex property of @p d also includes a list of the reverse edges.
/// @param s A @ref graph_view
/// @param d The destination @ref graph_view
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename Graph1, typename Graph2>
void copy_graph_struct_w_preds(Graph1 const& s, Graph2& d)
{
  //TODO: right now, the algorithm only works on static graphs
  //      uncomment the line below when dynamic graph scales correctly
  //map_func(add_verts_wf(), s, make_repeat_view(d));  //first add vertices
  map_func(add_all_edges_wf(), s, make_repeat_view(d));  //and then edges
}


//////////////////////////////////////////////////////////////////////
/// @brief Performs an asynchronous BFS traversal over edges until
/// the add_mark() method of the property returns false.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VertexGIDType>
struct pscc_async_bfs
{
 public:
  typedef pscc_async_bfs<VertexGIDType>           this_type;
  typedef VertexGIDType                           color_type;
  typedef void                                    result_type;

 private:
  color_type  m_mark;
  bool        m_forward;

 public:
  pscc_async_bfs(VertexGIDType mark, bool forward)
    : m_mark(mark),
      m_forward(forward)
  { }

  template <typename TGV, typename Vertex, typename Graph>
  result_type operator()(TGV tgv, Vertex v, Graph g)
  {
    typedef typename Vertex::property_type                property_type;
    typedef typename property_type::reachables_iterator   pred_iterator;

    if (!v.property().add_mark(m_mark, m_forward)) {
      return;
    }

    if (m_forward) {
      for (auto&& i : v) {
        tgv.add_task(*this, localize_ref(tgv), localize_ref(g, i.target()),
                     localize_ref(g));
      }
    } else {
      property_type prop = v.property();
      for (auto&& i : prop) {
        tgv.add_task(*this, localize_ref(tgv), localize_ref(g, i),
                     localize_ref(g));
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_mark);
    t.member(m_forward);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compares the signature of node u to that of node v. If the
/// signatures differ, or either of them is invalid, the edge (and the
/// reverse edge between the two nodes is broken. Note that the type of
/// signature depends on the pSCC algorithm in use.
/// @tparam Graph Graph type.
/// @tparam VD The type of the vertex descriptor.
/// @tparam NodeSignature Signature type for distinguishing node from neighbors.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename Graph, typename VD, typename NodeSignature>
struct pscc_check_and_remove
{
private:
  p_object_pointer_wrapper<Graph> m_g;
  VD                              m_u;
  VD                              m_v;
  NodeSignature                   m_usig;

public:
  typedef void result_type;

  pscc_check_and_remove(Graph& g, VD u, VD v, NodeSignature const& usig)
    : m_g(&g),
      m_u(u),
      m_v(v),
      m_usig(usig)
  { }

  template <typename Property>
  void operator()(Property& p) const
  {
    if (m_usig == Property::invalid_signature() || m_usig !=p.get_signature()) {
      p.remove_predecessor(m_u);
      m_g->delete_edge(typename Graph::edge_descriptor(m_u, m_v));
    }
  }

  void define_type(typer& t)
  {
    t.member(m_g);
    t.member(m_u);
    t.member(m_v);
    t.member(m_usig);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Removes all edges whose end nodes do not have the same signature
///
/// Iterates over every edge in v, calling @ref pscc_check_and_remove
/// on every source/target pair.
/// @param v The vertex.
/// @param g The @ref graph_view.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename Vertex, typename GView>
void pscc_separate_disjoint_func(Vertex v, GView g)
{

  typedef typename Vertex::vertex_descriptor              vertex_descriptor;
  typedef typename Vertex::property_type::mark_signature  signature_type;
  typedef pscc_check_and_remove<
            typename view_traits<GView>::container,
            vertex_descriptor,
            signature_type
          >                                               wf_type;

  signature_type my_signature = v.property().get_signature();
  vertex_descriptor vd = v.descriptor();

  for (auto&& i : v) {
    g.vp_apply_async(i.target(),
                     wf_type(g.container(), vd, i.target(), my_signature));
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Wraps two non-overlapping functions into one to avoid syncs
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_separate_disjoint_return_successful
{
  typedef size_t   result_type;

  //////////////////////////////////////////////////////////////////////
  /// @return Whether this is a successful pivot.
  ///
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex, typename GView>
  result_type operator()(Vertex v, GView g)
  {
    if (v.size()) {
      pscc_separate_disjoint_func(v,g);
    }

    return v.property().is_pivot() && v.property().get_cc() == v.descriptor();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wraps two non-overlapping functions into one to avoid syncs.
/// @return Whether this is not yet in an scc.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_separate_disjoint_return_uncolored
{
  typedef size_t   result_type;

  template <typename Vertex, typename GView>
  result_type operator()(Vertex v, GView g)
  {
    if (v.size()) {
      pscc_separate_disjoint_func(v,g);
    }

    return (v.property().has_valid_cc() ? 0 : 1);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Copies all the sccs back to the original graph using the
/// @c set_cc() method of the property.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct copy_sccs_wf
{
  typedef void result_type;

  template <typename V1, typename V2>
  void operator()(V1 c, V2 o)
  {
    /// @todo Use a property map instead of a property.
    o.property().set_cc(c.property().get_cc());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Removes a predecessor from a property. Used in
/// @ref graph_view::vp_apply_async()
/// calls.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VD>
struct pscc_remove_pred
{
 private:
  VD m_u;

 public:
  typedef void      result_type;

  pscc_remove_pred(VD u)
    : m_u(u)
  { }

  template <typename Property>
  result_type operator()(Property& p) const
  {
    p.remove_predecessor(m_u);
  }

  void define_type(typer& t)
  {
    t.member(m_u);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Traverses the graph from all source nodes, removing their edges
/// and then executing itself on all of their former neighbors.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_trim_forward
  : public dynamic_wf
{
  typedef void      result_type;

  template <typename TGV, typename Vertex, typename Graph>
  result_type operator()(TGV tgv, Vertex v, Graph g) const
  {
    typedef typename Vertex::property_type        property_type;
    typedef typename Vertex::vertex_descriptor    vertex_descriptor;
    typedef pscc_remove_pred<vertex_descriptor>   remove_pred;

    property_type prop = v.property();
    if (prop.begin() == prop.end() && !v.property().has_valid_cc()) {
      v.property().set_cc(v.descriptor());

      typename Vertex::adj_edges_type edges = v.edges();
      v.clear();

      vertex_descriptor vd = v.descriptor();
      for (auto&& i : edges) {
        g.vp_apply_async(i.target(), remove_pred(vd));
        tgv.add_task(*this, localize_ref(tgv), localize_ref(g, i.target()),
                     localize_ref(g));
      }
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Traverses the graph from all sink nodes, removing their edges
/// and then executing itself on all of their former neighbors.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct pscc_trim_backward
  : public dynamic_wf
{
  typedef void result_type;

  template <typename TGV, typename Vertex, typename Graph>
  result_type operator()(TGV tgv, Vertex v, Graph g) const
  {
    typedef typename Vertex::property_type                property_type;
    typedef typename property_type::reachables_iterator   reachables_iterator;
    typedef typename Vertex::vertex_descriptor            vertex_descriptor;

    if (v.size() == 0 && !v.property().has_valid_cc()) {
      v.property().set_cc(v.descriptor());

      property_type prop = v.property();
      v.property().clear_predecessors();

      vertex_descriptor vd = v.descriptor();
      for (auto&& i : prop) {
        g.delete_edge(typename Graph::edge_descriptor(i, vd));
        tgv.add_task(*this, localize_ref(tgv),
          localize_ref(g, i), localize_ref(g));
      }
    }
  }

};

} // namespace algo_details

} // namespace stapl

#endif

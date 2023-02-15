/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_CREATE_LEVEL_PARTIAL_INFO_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_CREATE_LEVEL_PARTIAL_INFO_HPP

#include <stapl/containers/graph/algorithms/create_level.hpp>

namespace stapl {

namespace create_level_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to mark vertices with their initial matchings, as
/// assigned by @ref set_roots_wf.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename VertexDesc>
struct mark_if
{
  VertexDesc m_source;
  VertexDesc m_target;

  mark_if(VertexDesc const& source, VertexDesc const& target)
    : m_source(source), m_target(target)
  { }

  typedef void result_type;

  template<typename Property>
  void operator()(Property& p) const
  {
    if (m_source == p)
      p = m_target;
  }

  void define_type(typer& t)
  {
    t.member(m_source);
    t.member(m_target);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to select matchings for each vertex before running
/// the pointer-jumping algorithm. Breaks cycles in the initial matchings,
/// if any.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct set_roots_wf
{
  typedef void result_type;

  template<typename Vertex, typename PMap>
  void operator()(Vertex v, PMap& pmap)
  {
    typedef typename Vertex::vertex_descriptor vertex_descriptor_t;
    typename PMap::value_type root = pmap.get(v);
    if (root > v.descriptor())
      pmap.apply(root, mark_if<vertex_descriptor_t>(v.descriptor(), root));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to select matchings for each vertex before running
/// the pointer-jumping algorithm. Breaks cycles in the initial matchings,
/// if any.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename View, typename PMap>
void set_roots(View gvw, PMap& pmap)
{ nc_map_func(set_roots_wf(), gvw, make_repeat_view(pmap)); }


//////////////////////////////////////////////////////////////////////
/// @brief Work-function for the naive pointer-jumping algorithm.
/// @ingroup pgraphAlgoDetails
///
/// Updates each vertex's parent to its parent's parent.
/// @todo Use a better pointer-jumping algorithm to make this phase
/// faster.
//////////////////////////////////////////////////////////////////////
struct redirect_functor
{
  typedef void result_type;

  template<typename Vertex, typename View, typename PMap>
  void operator()(Vertex v, View& vw, PMap& pmap)
  {
    typename Vertex::vertex_descriptor vert = pmap.get(v);
    typename Vertex::vertex_descriptor x = pmap.get(vert);
    if (vert != v.descriptor() && v.size() != 0) {
      while (x != vert) {  // while I don't reach a root.
        pmap.put(v, x);    // set my parent to parent's parent.
        vert = x;
        x = pmap.get(vert);
      }
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A naive pointer-jumping algorithm to find out the final
/// vertex grouping based on partial information.
/// @ingroup pgraphAlgoDetails
///
/// At the end of this algorithm, each vertex has its group-ID set
/// to a leader-vertex's descriptor in the provided property-map,
/// based on the input information in the same map.
/// @param gvw The input @ref graph_view.
/// @param p The input/output vertex property map storing the initial
/// leader-vertex for each vertex. This initial leader-vertex will be
/// updated to the final leader-vertex based on the pointer-jumping
/// algorithm.
/// @todo Use a better pointer-jumping algorithm to make this phase
/// faster.
//////////////////////////////////////////////////////////////////////
template<typename View, typename PMap>
void pointer_jumping(View& gvw, PMap& p)
{ nc_map_func(redirect_functor(), gvw, make_repeat_view(gvw), make_repeat_view(p)); }

} // namespace create_level_detail.



//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to create an entire level of hierarchy at once,
/// based on partial vertex-matching information.
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
/// @param partial_vertex_group_map Vertex property map identifying the "group"
/// of each vertex in the graph. The leader-vertex of each group is the
/// vertex whose group ID is the same as its descriptor. There must be
/// exactly one leader vertex in each group. Non-leader vertices may choose
/// to "collapse" with other leader or non-leader vertices. The final grouping
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
/// @return A graph_view over the output supergraph. The type of this view
/// is the same as the input graph_view.
//////////////////////////////////////////////////////////////////////
template<typename GraphView, typename PropMap,
         typename VertexPReducer, typename EdgePReducer>
GraphView create_level_partial_info(GraphView& gvw,
                                    PropMap& partial_vertex_group_map,
                                    VertexPReducer vpr, EdgePReducer epr,
                                    size_t max_msg_sz = 512,
                                    bool self_edges = false)
{
  // mark pairs destined to contract with each other -- one of them as root.
  create_level_detail::set_roots(gvw, partial_vertex_group_map);

  // perform pointer-jumping to figure out the actual complete groupings.
  create_level_detail::pointer_jumping(gvw, partial_vertex_group_map);

  // now we have the complete matching information, so we can call create_level.
  return create_level(gvw, partial_vertex_group_map, vpr, epr,
                      max_msg_sz, self_edges);
}

}  // namespace stapl

#endif

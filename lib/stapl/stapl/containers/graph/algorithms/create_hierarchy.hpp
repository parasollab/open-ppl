/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_CREATE_HIERARCHY_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_CREATE_HIERARCHY_HPP

#include <stapl/containers/graph/algorithms/create_level.hpp>
#include <stapl/containers/graph/algorithms/create_level_partial_info.hpp>
#include <stapl/views/repeated_view.hpp>

namespace stapl {

namespace create_hierarchy_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to copy vertex properties from supervertices in the
/// hierarchy to a flat graph, as well as to add corresponding edges
/// between vertices of the flat graph to reflect the parent-child
/// relations in the hierarchy.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct copy_vertex_wf
{
  size_t m_next_level_start;

  copy_vertex_wf(size_t const& next_level_start)
    : m_next_level_start(next_level_start)
  { }

  typedef void result_type;

  template<typename VertexH, typename VertexO, typename FlatGraph>
  void operator()(VertexH v_hierarchy, VertexO v_output,
                  FlatGraph flat_graph) const
  {
    // Copy the property.
    v_output.property() = v_hierarchy.property().property;
    // Add edges.
    for (size_t i=0; i < v_hierarchy.property().children.size(); ++i) {
      const size_t child_desc = v_hierarchy.property().children[i] +
                                m_next_level_start;

      flat_graph.add_edge_async(v_output.descriptor(), child_desc, true);

      flat_graph.add_edge_async(child_desc, v_output.descriptor(), false);
    }
  }

  void define_type(typer& t)
  { t.member(m_next_level_start); }
};

} // namespace create_hierarchy_detail.



//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to create multiple levels of hierarchy at once.
/// @ingroup pgraphAlgo
///
/// Creates multiple levels of hierarchy based on the input @ref graph_view
/// and the provided vertex-grouping algorithm. Supervertices and superedges in
/// each level store descriptors of their children in the level below.
/// Properties of supervertices and superedges are reductions of the properties
/// of their children through the user provided reducers.
/// @param gvw The input @ref graph_view over which the new levels are to be
/// created. The underlying graph container must be DIRECTED, NONMULTIEDGES,
/// and store the custom @ref super_vertex_property and
/// @ref super_edge_property on their vertices and edges, respectively.
/// @param coarsener A functor which takes an input @ref graph_view and level-id
/// and outputs an std::pair of bool indicating if the matching information
/// is complete (true) or partial (false), and a vertex property map
/// identifying the "group" of each vertex in the graph. The leader-vertex of
/// each group is the vertex whose group ID is the same as its descriptor.
/// There must be exactly one leader vertex in each group. Non-leader vertices
/// may choose to "collapse" with other leader or non-leader vertices. The
/// final grouping with which to create the supervertex is calculated by running
/// a pointer-jumping (or similar) algorithm. [vertex->vertex_descriptor]
/// @param vpr The vertex property reducer for reducing child-vertex
/// properties to form the supervertex property.
/// @param epr The edge property reducer for reducing child-edge
/// properties to form the superedge property.
/// @param done A functor which takes an input @ref graph_view and level-id
/// and outputs if another level of hierarchy should be created. [bool]
/// @param max_msg_sz The number of requests to aggregate for the aggregator.
/// @param self_edges Indicates whether self-edges are allowed on the output
/// graph or not. If the supergraph has self-edges, they will represent the
/// internal edges between the supervertex's child vertices.
/// @param sort_edges Indicates if the adjacent-edges for each supervertex
/// in the output supergraph will be sorted based on their target-descriptors.
/// @return An std::vector of @ref graph_view representing the final hierarchy.
/// The type of the view is the same as the input graph_view.
/// Size of the output reflects the number of levels in the hierarchy, and
/// hierarchy[i] represents the graph at the i-th level in the hierarchy,
/// with hierarchy[0] being the original input graph.
//////////////////////////////////////////////////////////////////////
template<typename GraphView, typename Coarsener,
         typename VertexPReducer, typename EdgePReducer, typename Done>
std::vector<GraphView>
  create_hierarchy(GraphView& gvw,
                   Coarsener const& coarsener,
                   VertexPReducer const& vpr, EdgePReducer const& epr,
                   Done const& done,
                   size_t max_msg_sz = 512,
                   bool self_edges = false, bool sort_edges = false)
{
  using namespace create_hierarchy_detail;

  std::vector<GraphView> hierarchy;
  // The input graph view forms level-0 of the hierarchy.
  hierarchy.push_back(gvw);

  size_t level = 0;
  while (hierarchy[level].size() > 1 && !done(hierarchy[level], level)) {
    // coarsen the graph at this level to create matchings for the next level,
    // then we will have the complete matching information, so we can
    // create the level and add it to the hierarchy.
    hierarchy.push_back(create_level(hierarchy[level],
                                     coarsener(hierarchy[level], level),
                                     vpr, epr,
                                     max_msg_sz, self_edges));
    // now we're ready to process the next level of the hierarchy.
    ++level;
  }

  return hierarchy;
}



//////////////////////////////////////////////////////////////////////
/// @brief Algorithm to flatten multiple levels of hierarchy into a single
/// level.
///
/// The edges of the output graph represent parent-child relationship in the
/// hierarchy. The leaf-vertices represent the vertices of level-0 of the
/// hierarchy and non-leaf vertices represent supervertices in
/// intermediate-levels of the hierarchy.
/// @ingroup pgraphAlgo
///
/// @param hierarchy An std::vector of @ref graph_view representing the
/// hierarchy.
/// Size of the input reflects the number of levels in the hierarchy, and
/// hierarchy[i] represents the graph at the i-th level in the hierarchy,
/// with hierarchy[0] being the lowest level in the hierarchy.
/// @return A @ref graph_view representing the flattened hierarchy.
/// The view is over a DIRECTED, NONMULTIEDGES static graph whose vertices
/// have the user vertex-property stored on them and edge-property is a bool
/// indicating if it is an edge from a vertex in a higher-level to a vertex
/// in a lower-level (true) or vice-versa (false).
/// @todo The naive fill algorithm may not distribute the work most effectively,
/// we may want to find a better algorithm to improve distribute vertices.
//////////////////////////////////////////////////////////////////////
template<typename GraphView>
graph_view<graph<DIRECTED, NONMULTIEDGES,
                 typename GraphView::vertex_property::property_type, bool> >
flatten_hierarchy(std::vector<GraphView> const& hierarchy)
{
  using namespace create_hierarchy_detail;

  size_t total_vertices = 0;
  for (size_t i=0; i<hierarchy.size(); ++i)
    total_vertices += hierarchy[i].size();

  typedef graph<DIRECTED, NONMULTIEDGES,
                typename GraphView::vertex_property::property_type,
                bool>                                                   graph_t;
  typedef graph_view<graph_t>                                      graph_view_t;
  typedef typename graph_view_t::domain_type                       dom_t;
  graph_t* flat_graph = new graph_t(total_vertices);
  graph_view_t flat_graph_view(flat_graph);

  // naive fill: this probably won't distribute the work
  // most effectively, but it's a start.
  size_t number_of_vertices_filled = 0;
  for (int i=hierarchy.size()-1; i>=0; --i) {
    const size_t curr_size = hierarchy[i].size();
    map_func(copy_vertex_wf(number_of_vertices_filled + curr_size),
             hierarchy[i],
             graph_view_t(*flat_graph,
                          dom_t(number_of_vertices_filled,
                                number_of_vertices_filled + curr_size - 1)),
             make_repeat_view(flat_graph_view));
    number_of_vertices_filled += curr_size;
  }

  return flat_graph_view;
}

}  // namespace stapl

#endif

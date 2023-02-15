/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_BGL_UNDIRECTED_GRAPH_ADAPTER_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_BGL_UNDIRECTED_GRAPH_ADAPTER_HPP

#include "values.h"
#include "graph.h"
#include <boost/config.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>


//////////////////////////////////////////////////////////////////////
/// @file
/// This file provides BGL-style wrapper functions for the
/// STAPL sequential graph to allow users to call BGL algorithms on it.
/// The functions in this file are specialized for the Undirected
/// graph. For directed graphs, please refer to
/// @ref bgl_directed_graph_adapter.hpp.
///
/// The functions and classes in this file allows the user to treat a
/// STAPL GRAPH object as a boost graph "as is". No wrapper is needed
/// for the GRAPH object.
///
/// @todo Create a helper struct that accepts the vertex and edge types
/// and reflectes the graph_traits type.  This will shorten the types
/// for each parameter significantly.
///
/// @note Include this header file before any Boost algorithm files.
///
/// @warning This implementation relies on partial specialization for
/// the graph_traits class (so it won't compile with Visual C++).
///
/// @todo Add tests for these methods.
//////////////////////////////////////////////////////////////////////


#if !defined BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION
namespace boost {

struct stapl_graph_traversal_category
  : public virtual bidirectional_graph_tag,
    public virtual adjacency_graph_tag,
    public virtual edge_list_graph_tag
{
};


template <class VertexProperty, class EdgeProperty>
struct graph_traits<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
{
  typedef typename stapl::sequential::graph<stapl::UNDIRECTED,
      stapl::MULTIEDGES, VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty> > graph;

  typedef typename graph::vertex_descriptor vertex_descriptor;
  typedef typename graph::edge_descriptor edge_descriptor;
  typedef typename graph::const_adj_edge_iterator adjacency_iterator;
  typedef typename graph::const_adj_edge_iterator out_edge_iterator;
  typedef typename graph::const_adj_edge_iterator in_edge_iterator;
  typedef typename graph::const_vertex_iterator vertex_iterator;
  typedef typename graph::const_edge_iterator edge_iterator;
  typedef typename graph::vertex_property vertex_property_type;
  typedef typename graph::edge_property edge_property_type;

  // typedef EdgeProperty edge_property_type;

  typedef edge_list_graph_tag graph_category;

  typedef directed_tag directed_category;  //directed
  typedef allow_parallel_edge_tag edge_parallel_category;  // multiedges
  typedef stapl_graph_traversal_category traversal_category;  // traversals
  typedef size_t vertices_size_type;
  typedef size_t edges_size_type;
  typedef size_t degree_size_type;
};

} // namespace boost
#endif


namespace boost {

//===========================================================================
// functions for GRAPH<VertexProperty,EdgeProperty>

//////////////////////////////////////////////////////////////////////
/// @brief Returns the @p v_int_id-th vertex in the specified graph.
/// @ingroup graphInterop
/// @param v_int_id The position, from start of graph, of the vertex of
/// interest.
/// @param g A STAPL sequential graph.
/// @return The vertex_descriptor of the vertex at the specified position
/// from the start of the graph, in graph @p g.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
typename graph_traits<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
              ::vertex_descriptor
vertex(
    int v_int_id,
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  typedef typename graph_traits<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                ::vertex_iterator Iter;
  int j = 0;
  for (Iter i = g.begin(); i != g.end(); ++i, ++j)
    if (j == v_int_id)
      return i.descriptor();
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns the vertex descriptor of the source vertex of the specified
/// edge in the provided graph.
/// @ingroup graphInterop
/// @param e The descriptor of the specified edge of a STAPL sequential graph.
/// @param g A STAPL sequential graph.
/// @return The vertex_descriptor of the source vertex of the specified edge.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
typename graph_traits<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
              ::vertex_descriptor
source(
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::edge_descriptor e,
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  return e.source();
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns the vertex descriptor of the target vertex of the specified
/// edge in the provided graph.
/// @ingroup graphInterop
/// @param e The descriptor of the specified edge of a STAPL sequential graph.
/// @param g A STAPL sequential graph.
/// @return The vertex_descriptor of the target vertex of the specified edge.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
typename graph_traits<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
              ::vertex_descriptor
target(
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::edge_descriptor e,
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  return e.target();
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns a pair of iterators over the vertices of the specified
/// STAPL sequential graph.
/// @ingroup graphInterop
/// @param g A STAPL sequential graph.
/// @return A pair of iterators over the vertices of the input graph.
/// The first of the pair points to the start vertex of the graph, while
/// the second of the pair points to one-past the end vertex of the graph.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
std::pair<
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_iterator,
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_iterator>
vertices(
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  typedef typename graph_traits<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                ::vertex_iterator Iter;
  return std::make_pair(Iter(g.begin()), Iter(g.end()));
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns a pair of iterators over the edges of the specified
/// STAPL sequential graph.
/// @ingroup graphInterop
/// @param g A STAPL sequential graph.
/// @return A pair of iterators over the edges of the input graph.
/// The first of the pair points to the start edge of the graph, while
/// the second of the pair points to one-past the end edge of the graph.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
std::pair<
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::edge_iterator,
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::edge_iterator>
edges(
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  typedef typename graph_traits<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                ::edge_iterator Iter;
  return std::make_pair(Iter(g.edges_begin()), Iter(g.edges_end()));
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns a pair of iterators over the out-edges of the specified
/// vertex in the specified STAPL sequential graph.
/// @ingroup graphInterop
/// @param u The vertex descriptor of the specified vertex.
/// @param g A STAPL sequential graph.
/// @return A pair of iterators over the out-edges of the specified vertex
/// of the input graph.
/// The first of the pair points to the start out-edge of the vertex, while
/// the second of the pair points to one-past the end out-edge of the vertex.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
std::pair<
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::out_edge_iterator,
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::out_edge_iterator>
out_edges(
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_descriptor u,
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  typedef typename graph_traits<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                ::out_edge_iterator Iter;
  typename graph_traits<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                ::vertex_iterator vi = g.find_vertex(u);
  return std::make_pair(Iter(vi.begin()), Iter(vi.end()));
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns a pair of iterators over the in-edges of the specified
/// vertex in the specified STAPL sequential graph.
/// @ingroup graphInterop
/// @param u The vertex descriptor of the specified vertex.
/// @param g A STAPL sequential graph.
/// @return A pair of iterators over the in-edges of the specified vertex
/// of the input graph.
/// The first of the pair points to the start in-edge of the vertex, while
/// the second of the pair points to one-past the end in-edge of the vertex.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
std::pair<
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::in_edge_iterator,
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::in_edge_iterator>
in_edges(
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_descriptor u,
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  typedef typename graph_traits<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                ::in_edge_iterator Iter;
  return std::make_pair(Iter(u.begin()), Iter(u.end()));
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns a pair of iterators over the adjacents of the specified
/// vertex in the specified STAPL sequential graph.
/// @ingroup graphInterop
/// @param u The vertex descriptor of the specified vertex.
/// @param g A STAPL sequential graph.
/// @return A pair of iterators over the adjacents of the specified vertex
/// of the input graph.
/// The first of the pair points to the start adjacent of the vertex, while
/// the second of the pair points to one-past the end adjacent of the vertex.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
std::pair<
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::adjacency_iterator,
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::adjacency_iterator>
adjacent_vertices(
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_descriptor u,
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  typedef typename graph_traits<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                ::adjacency_iterator Iter;
  return std::make_pair(Iter(u.begin(), &g), Iter(u.end(), &g));
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns the number of vertices in the specified STAPL sequential
/// graph.
/// @ingroup graphInterop
/// @param g A STAPL sequential graph.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
typename graph_traits<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
              ::vertices_size_type
num_vertices(
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  return g.get_num_vertices();
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns the number of edges in the specified STAPL sequential
/// graph.
/// @ingroup graphInterop
/// @param g A STAPL sequential graph.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
typename graph_traits<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
              ::edges_size_type
num_edges(
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  return g.get_num_edges();
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns the number of out-edges of the specified vertex in
/// the input STAPL sequential graph.
/// @ingroup graphInterop
/// @param u The vertex descriptor of the specified vertex.
/// @param g A STAPL sequential graph.
/// @return The number of out-edges of the specified vertex in the input
/// graph.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
typename graph_traits<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
              ::degree_size_type
out_degree(
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_descriptor u,
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  return (*g.find_vertex(u)).size();
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns the number of in-edges of the specified vertex in
/// the input STAPL sequential graph.
/// @ingroup graphInterop
/// @param u The vertex descriptor of the specified vertex.
/// @param g A STAPL sequential graph.
/// @return The number of in-edges of the specified vertex in the input
/// graph.
/// @bug Function returns out-degree instead of in-degree of the vertex.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
typename graph_traits<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
              ::degree_size_type
in_degree(
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_descriptor u,
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  return (*g.find_vertex(u)).size();
}


//////////////////////////////////////////////////////////////////////
/// @brief Returns the number of edges of the specified vertex in
/// the input STAPL sequential graph.
/// @ingroup graphInterop
/// @param u The vertex descriptor of the specified vertex.
/// @param g A STAPL sequential graph.
/// @return The number of edges of the specified vertex in the input
/// graph.
/// @bug Function returns out-degree instead of total degree of the vertex.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
typename graph_traits<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
              ::degree_size_type
degree(
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_descriptor u,
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  return (*g.find_vertex(u)).size();
}


//////////////////////////////////////////////////////////////////////
/// @brief Adds a vertex to the specified graph, returning its descriptor.
/// Uses default property.
/// @ingroup graphInterop
/// @param g A STAPL sequential graph.
/// @return The descriptor of the added vertex.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
typename graph_traits<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
              ::vertex_descriptor
add_vertex(
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  return g.add_vertex();
}


//////////////////////////////////////////////////////////////////////
/// @brief Adds a vertex with the specified property to the specified graph,
/// returning its descriptor.
/// @ingroup graphInterop
/// @param vp The vertex property of the vertex to be added.
/// @param g A STAPL sequential graph.
/// @return The descriptor of the added vertex.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
typename graph_traits<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
              ::vertex_descriptor
add_vertex(
    const VertexProperty& vp,
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  return g.add_vertex(vp);
}


//////////////////////////////////////////////////////////////////////
/// @brief Deletes all edges to and from the specified vertex.
/// @ingroup graphInterop
/// @param u The descriptor of the vertex.
/// @param g A STAPL sequential graph.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
void clear_vertex(
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_descriptor u,
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  typename graph_traits<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                ::vertex_iterator vi = g.find_vertex(u);
  typename graph_traits<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                ::out_edge_iterator ei;
  for (ei = vi.begin(); ei != vi.end(); ++ei)
    g.delete_edge(ei.descriptor());

  typename graph_traits<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                ::edge_iterator iei;
  for (iei = g.edges_begin(); iei != g.edges_end(); ++iei)
    if (iei.target() == u || iei.source() == u)
      g.delete_edge(iei.descriptor());
}


//////////////////////////////////////////////////////////////////////
/// @brief Deletes the specified vertex.
/// @ingroup graphInterop
/// @param u The descriptor of the vertex.
/// @param g A STAPL sequential graph.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
void remove_vertex(
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_descriptor u,
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  g.delete_vertex(u);
}


//////////////////////////////////////////////////////////////////////
/// @brief Adds an edge between two specified vertices.
/// @ingroup graphInterop
/// @param u The descriptor of the source vertex.
/// @param v The descriptor of the target vertex.
/// @param g A STAPL sequential graph.
/// @return pair<edge_descriptor, bool> Edge descriptor is the added edge
/// and the bool is true if the edge was added, or false otherwise.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
std::pair<
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::edge_descriptor, bool>
add_edge(
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_descriptor u,
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_descriptor v,
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  return std::make_pair(g.add_edge(u, v), true);  // not sure.
}


//////////////////////////////////////////////////////////////////////
/// @brief Adds an edge with the given property between two specified vertices.
/// @ingroup graphInterop
/// @param u The descriptor of the source vertex.
/// @param v The descriptor of the target vertex.
/// @param et The property of the edge.
/// @param g A STAPL sequential graph.
/// @return pair<edge_descriptor, bool> Edge descriptor is the added edge
/// and the bool is true if the edge was added, or false otherwise.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
std::pair<
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::edge_descriptor, bool>
add_edge(
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_descriptor u,
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_descriptor v,
    const EdgeProperty& et,
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  return std::make_pair(g.add_edge(u, v, et), true);  // not sure.
}


//////////////////////////////////////////////////////////////////////
/// @brief Deletes the edge with between the two specified vertices.
/// @ingroup graphInterop
/// @param u The descriptor of the source vertex.
/// @param v The descriptor of the target vertex.
/// @param g A STAPL sequential graph.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
void remove_edge(
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_descriptor u,
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::vertex_descriptor v,
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  typename graph_traits<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                ::vertex_iterator vi = g.find_vertex(u);
  typename graph_traits<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                ::out_edge_iterator i;
  for (i = vi.begin(); i != vi.end(); ++i)
    if (i.target() == v)
      g.delete_edge(i.descriptor());
}


//////////////////////////////////////////////////////////////////////
/// @brief Deletes the edge with the specified edge descriptor.
/// @ingroup graphInterop
/// @param e The descriptor of the edge.
/// @param g A STAPL sequential graph.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
void remove_edge(
    typename graph_traits<
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> > >
                  ::edge_descriptor e,
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  g.delete_edge(e);
}


//===========================================================================
// property maps for GRAPH<VertexProperty,EdgeProperty>

//////////////////////////////////////////////////////////////////////
/// @brief Vertex property map for BGL that returns identity.
/// @ingroup graphInterop
/// @tparam VertexProperty The vertex property type.
/// @tparam EdgeProperty The edge property type.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
class stapl_graph_id_map
  : public put_get_helper<int, stapl_graph_id_map<VertexProperty,
                          EdgeProperty> >
{
public:
  typedef readable_property_map_tag category;
  typedef int value_type;
  typedef int reference;
  typedef typename stapl::sequential::graph<stapl::UNDIRECTED,
      stapl::MULTIEDGES, VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty> >::vertex_descriptor key_type;

  stapl_graph_id_map() = default;

  template <class T>
  long operator[](T x) const
  {
    return x;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Edge property map for BGL that returns identity (edge-id).
/// @ingroup graphInterop
/// @tparam VertexProperty The vertex property type.
/// @tparam EdgeProperty The edge property type.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
class stapl_graph_edge_id_map
  : public put_get_helper<int, stapl_graph_edge_id_map<VertexProperty,
                          EdgeProperty> >
{
public:
  typedef readable_property_map_tag category;
  typedef int value_type;
  typedef int reference;
  typedef typename stapl::sequential::graph<stapl::UNDIRECTED,
      stapl::MULTIEDGES, VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty> >::edge_descriptor key_type;

  stapl_graph_edge_id_map() = default;

  template <class T>
  long operator[](T x) const
  {
    return x.id();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Edge property map for BGL that returns the weights of edges
/// stored in an external weight-vector.
/// @ingroup graphInterop
/// @tparam VertexProperty The vertex property type.
/// @tparam EdgeProperty The edge property type.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
class stapl_graph_edge_wt_id_map
  : public put_get_helper<int, stapl_graph_edge_wt_id_map<VertexProperty,
                          EdgeProperty> >
{
  const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
      VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty> >* g;
  const std::vector<EdgeProperty>* w_vec;
public:
  typedef readable_property_map_tag category;
  typedef int value_type;
  typedef int reference;
  typedef typename stapl::sequential::graph<stapl::UNDIRECTED,
      stapl::MULTIEDGES, VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty> >::edge_descriptor key_type;

  stapl_graph_edge_wt_id_map(
      const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> >* g,
      const std::vector<EdgeProperty>* w_vec_)
      : g(g), w_vec(w_vec_)
  { }

  template <class T>
  double operator[](T x) const
  {
    return (*w_vec)[x.id()];
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function to get the identity vertex property map associated
/// with the specified graph, based on vertex descriptors, i.e., the property
/// map returns the vertex descriptor as the property.
/// @ingroup graphInterop
/// @tparam VertexProperty The vertex property type.
/// @tparam EdgeProperty The edge property type.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
stapl_graph_id_map<VertexProperty, EdgeProperty>
get(
    vertex_index_t,
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  return stapl_graph_id_map<VertexProperty, EdgeProperty>();
}


//////////////////////////////////////////////////////////////////////
/// @brief Function to get the identity edge property map associated
/// with the specified graph, based on edge-indices, i.e., the property
/// map returns the edge-index as the property.
/// @ingroup graphInterop
/// @tparam VertexProperty The vertex property type.
/// @tparam EdgeProperty The edge property type.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
stapl_graph_edge_id_map<VertexProperty, EdgeProperty>
get(
    edge_index_t,
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  return stapl_graph_edge_id_map<VertexProperty, EdgeProperty>();
}


//////////////////////////////////////////////////////////////////////
/// @brief Function to get the identity edge property map associated
/// with the specified graph, based on edge-weights, i.e., the property
/// map returns the edge-weight as the property.
/// @ingroup graphInterop
/// @tparam VertexProperty The vertex property type.
/// @tparam EdgeProperty The edge property type.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
stapl_graph_edge_wt_id_map<VertexProperty, EdgeProperty>
get(
    edge_weight_t,
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  return stapl_graph_edge_wt_id_map<VertexProperty, EdgeProperty>(g);
}


//////////////////////////////////////////////////////////////////////
/// @brief Class for property map over STAPL sequential graph.
/// @ingroup graphInterop
/// @tparam Tag Specifies the type of the map, and what elements
/// it maps to.
//////////////////////////////////////////////////////////////////////
template <class Tag>
struct stapl_property_map
{
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref stapl_property_map for vertex indices.
/// @ingroup graphInterop
//////////////////////////////////////////////////////////////////////
template <>
struct stapl_property_map<vertex_index_t>
{
  template <class VertexProperty, class EdgeProperty>
  struct bind_
  {
    typedef stapl_graph_id_map<VertexProperty, EdgeProperty> type;
    typedef stapl_graph_id_map<VertexProperty, EdgeProperty> const_type;
  };
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref stapl_property_map for edge indices.
/// @ingroup graphInterop
//////////////////////////////////////////////////////////////////////
template <>
struct stapl_property_map<edge_index_t>
{
  template <class VertexProperty, class EdgeProperty>
  struct bind_
  {
    typedef stapl_graph_edge_id_map<VertexProperty, EdgeProperty> type;
    typedef stapl_graph_edge_id_map<VertexProperty, EdgeProperty> const_type;
  };
};

//////////////////////////////////////////////////////////////////////
/// @brief Class for property map over STAPL sequential graph.
/// @ingroup graphInterop
///
/// Returns a reference to the value stored on the vertex of the graph.
/// @tparam Data The value type stored in the map.
/// @tparam DataRef The reference type for the value-type stored in the map.
/// @tparam GraphPtr The type of the pointer to the storage for this map.
/// This map uses this pointer to access the data for the map.
//////////////////////////////////////////////////////////////////////
template <class Data, class DataRef, class GraphPtr>
class stapl_graph_data_map
  : public put_get_helper<DataRef,
                          stapl_graph_data_map<Data, DataRef, GraphPtr> >
{
public:
  typedef Data value_type;
  typedef DataRef reference;
  typedef void key_type;
  typedef lvalue_property_map_tag category;

  stapl_graph_data_map(GraphPtr g)
      : m_g(g)
  { }

  template <class NodeOrEdge>
  DataRef operator[](NodeOrEdge x) const
  {
    return (*m_g)[x];
  }
protected:
  GraphPtr m_g;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref stapl_property_map for vertices, using
/// stored data.
/// @ingroup graphInterop
//////////////////////////////////////////////////////////////////////
template <>
struct stapl_property_map<vertex_all_t>
{
  template <class VertexProperty, class EdgeProperty>
  struct bind_
  {
    typedef stapl_graph_data_map<VertexProperty, VertexProperty&,
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> >*> type;
    typedef stapl_graph_data_map<VertexProperty, const VertexProperty&,
        const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> >*> const_type;
  };
};


//////////////////////////////////////////////////////////////////////
/// @brief Function to get the stored vertex property map associated
/// with the specified graph, i.e., the property map returns the data
/// stored corresponding to each vertex as the property.
/// @ingroup graphInterop
/// @tparam VertexProperty The vertex property type.
/// @tparam EdgeProperty The edge property type.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
typename property_map<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >, vertex_all_t>
              ::type
get(
    vertex_all_t,
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  typedef typename property_map<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> >, vertex_all_t>
                ::type pmap_type;
  return pmap_type(&g);
}


//////////////////////////////////////////////////////////////////////
/// @brief Function to get the stored vertex property map associated
/// with the specified const graph, i.e., the property map returns the data
/// stored corresponding to each vertex as the property.
/// @ingroup graphInterop
/// @tparam VertexProperty The vertex property type.
/// @tparam EdgeProperty The edge property type.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
typename property_map<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >, vertex_all_t>
              ::const_type
get(
    vertex_all_t,
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  typedef typename property_map<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> >,
              vertex_all_t>::const_type pmap_type;
  return pmap_type(&g);
}


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref stapl_property_map for all edges.
/// @ingroup graphInterop
//////////////////////////////////////////////////////////////////////
template <>
struct stapl_property_map<edge_all_t>
{
  template <class VertexProperty, class EdgeProperty>
  struct bind_
  {
    typedef stapl_graph_data_map<EdgeProperty, EdgeProperty&,
        stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> >*> type;
    typedef stapl_graph_data_map<EdgeProperty, const EdgeProperty&,
        const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
            VertexProperty, EdgeProperty,
            stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                stapl::MULTIEDGES, VertexProperty, EdgeProperty> >*>
                const_type;
  };
};


//////////////////////////////////////////////////////////////////////
/// @brief Function to get the stored edge property map associated
/// with the specified graph, i.e., the property map returns the data
/// stored corresponding to each edge as the property.
/// @ingroup graphInterop
/// @tparam VertexProperty The vertex property type.
/// @tparam EdgeProperty The edge property type.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
typename property_map<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >, edge_all_t>
              ::type
get(
    edge_all_t,
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  typedef typename property_map<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> >, edge_all_t>
                ::type pmap_type;
  return pmap_type(&g);
}


//////////////////////////////////////////////////////////////////////
/// @brief Function to get the stored edge property map associated
/// with the specified const graph, i.e., the property map returns the data
/// stored corresponding to each edge as the property.
/// @ingroup graphInterop
/// @tparam VertexProperty The vertex property type.
/// @tparam EdgeProperty The edge property type.
//////////////////////////////////////////////////////////////////////
template <class VertexProperty, class EdgeProperty>
typename property_map<
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >, edge_all_t>
              ::const_type
get(
    edge_all_t,
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g)
{
  typedef typename property_map<
      stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty,
          stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
              stapl::MULTIEDGES, VertexProperty, EdgeProperty> >, edge_all_t>
                ::const_type pmap_type;
  return pmap_type(&g);
}


/*
// property map interface to the STAPL node_array class

template <class E, class ERef, class NodeMapPtr>
class stapl_node_property_map
  : public put_get_helper<ERef, stapl_node_property_map<E, ERef, NodeMapPtr> >
{
public:
  typedef E value_type;
  typedef ERef reference;
  typedef typename stapl::sequential::graph<stapl::UNDIRECTED,
      stapl::MULTIEDGES, VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty> >::vertex_descriptor key_type;
  typedef lvalue_property_map_tag category;

  stapl_node_property_map(NodeMapPtr a)
    : m_array(a)
  { }

  ERef operator[](stapl::sequential::graph<stapl::UNDIRECTED,
                  stapl::MULTIEDGES, VertexProperty, EdgeProperty,
                  stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                  stapl::MULTIEDGES, VertexProperty, EdgeProperty> >
                    ::vertex_descriptor n) const
  {
    return (*m_array)[n];
  }

protected:
  NodeMapPtr m_array;
};


template <class E>
stapl_node_property_map<E, const E&, const stapl::sequential::graph<
    stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty, EdgeProperty,
    stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty> >::vertex_descriptor_array<E>*>
make_stapl_node_property_map(const stapl::sequential::graph<stapl::UNDIRECTED,
    stapl::MULTIEDGES, VertexProperty, EdgeProperty,
    stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty> >::vertex_descriptor_array<E>& a)
{
  typedef stapl_node_property_map<E, const E&, const stapl::sequential::graph<
      stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty> >::vertex_descriptor_array<E>*>
          pmap_type;
  return pmap_type(&a);
}


template <class E>
stapl_node_property_map<E, E&, stapl::sequential::graph<stapl::UNDIRECTED,
    stapl::MULTIEDGES, VertexProperty, EdgeProperty,
    stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty> >::vertex_descriptor_array<E>*>
make_stapl_node_property_map(stapl::sequential::graph<stapl::UNDIRECTED,
    stapl::MULTIEDGES, VertexProperty, EdgeProperty,
    stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty> >::vertex_descriptor_array<E>& a)
{
  typedef stapl_node_property_map<E, E&, stapl::sequential::graph<
      stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty> >::vertex_descriptor_array<E>*>
          pmap_type;
  return pmap_type(&a);
}


template <class E>
stapl_node_property_map<E, const E&, const stapl::sequential::graph<
    stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty, EdgeProperty,
    stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty> >::vertex_descriptor_map<E>*>
make_stapl_node_property_map(const stapl::sequential::graph<stapl::UNDIRECTED,
    stapl::MULTIEDGES, VertexProperty, EdgeProperty,
    stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty> >::node_map<E>& a)
{
  typedef stapl_node_property_map<E,const E&,const stapl::sequential::graph<
      stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty> >::node_map<E>*> pmap_type;
  return pmap_type(&a);
}


template <class E>
stapl_node_property_map<E, E&, stapl::sequential::graph<stapl::UNDIRECTED,
    stapl::MULTIEDGES, VertexProperty, EdgeProperty,
    stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty> >::node_map<E>*>
make_stapl_node_property_map(stapl::sequential::graph<stapl::UNDIRECTED,
    stapl::MULTIEDGES, VertexProperty, EdgeProperty,
    stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty> >::node_map<E>& a)
{
  typedef stapl_node_property_map<E, E&, stapl::sequential::graph<
      stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty> >::node_map<E>*> pmap_type;
  return pmap_type(&a);
}


// g++ 'enumeral_type' in template unification not implemented workaround
template <class VertexProperty, class EdgeProperty, class Tag>
struct property_map<stapl::sequential::graph<stapl::UNDIRECTED,
    stapl::MULTIEDGES, VertexProperty, EdgeProperty,
    stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty> >, Tag>
{
  typedef typename
  stapl_property_map<Tag>::template bind_<VertexProperty, EdgeProperty>
  map_gen;
  typedef typename map_gen::type type;
  typedef typename map_gen::const_type const_type;
};


template <class VertexProperty, class EdgeProperty, class PropertyTag,
    class Key>
typename boost::property_traits<
    typename boost::property_map<stapl::sequential::graph<stapl::UNDIRECTED,
        stapl::MULTIEDGES, VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >,
            PropertyTag>::const_type::value_type
get(
    PropertyTag p, const stapl::sequential::graph<stapl::UNDIRECTED,
        stapl::MULTIEDGES, VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
        stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g, const Key& key)
{
  return get(get(p, g), key);
}


template <class VertexProperty, class EdgeProperty, class PropertyTag,
    class Key, class Value>
void put(
    PropertyTag p, stapl::sequential::graph<stapl::UNDIRECTED,
        stapl::MULTIEDGES, VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >& g,
const Key& key, const Value& value)
{
  typedef typename property_map<stapl::sequential::graph<stapl::UNDIRECTED,
      stapl::MULTIEDGES, VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty> >, PropertyTag>::type Map;
  Map pmap = get(p, g);
  put(pmap, key, value);
}

// property map interface to the STAPL edge_array class

template <class E, class ERef, class EdgeMapPtr>
class stapl_edge_property_map
  : public put_get_helper<ERef, stapl_edge_property_map<E, ERef, EdgeMapPtr> >
{
public:
  typedef E value_type;
  typedef ERef reference;
  typedef stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
      VertexProperty, EdgeProperty, stapl::sequential::adj_graph_traits<
          stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty, EdgeProperty> >
            ::edge_descriptor key_type;
  typedef lvalue_property_map_tag category;

  stapl_edge_property_map(EdgeMapPtr a) : m_array(a) { }

  ERef operator[](stapl::sequential::graph<stapl::UNDIRECTED,
                  stapl::MULTIEDGES, VertexProperty, EdgeProperty,
                  stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
                  stapl::MULTIEDGES, VertexProperty, EdgeProperty> >
                    ::edge_descriptor n) const
  {
    return (*m_array)[n];
  }

protected:
  EdgeMapPtr m_array;
};


template <class E>
stapl_edge_property_map<E, const E&, const stapl::sequential::graph<
    stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty, EdgeProperty,
    stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty> >::edge_array<E>*>
make_stapl_node_property_map(
    const stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty, stapl::sequential::adj_graph_traits<
            stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty, EdgeProperty
              > >::node_array<E>& a)
{
  typedef stapl_edge_property_map<E, const E&, const stapl::sequential::graph<
      stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty> >::node_array<E>*> pmap_type;
  return pmap_type(&a);
}


template <class E>
stapl_edge_property_map<E, E&, stapl::sequential::graph<stapl::UNDIRECTED,
    stapl::MULTIEDGES, VertexProperty, EdgeProperty,
    stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty> >::edge_array<E>*>
make_stapl_edge_property_map(
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty,
        stapl::sequential::adj_graph_traits<stapl::UNDIRECTED,
            stapl::MULTIEDGES, VertexProperty, EdgeProperty> >
              ::edge_array<E>& a)
{
  typedef stapl_edge_property_map<E, E&, stapl::sequential::graph<
      stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
      VertexProperty, EdgeProperty> >::edge_array<E>*> pmap_type;
  return pmap_type(&a);
}


template <class E>
stapl_edge_property_map<E, const E&, const stapl::sequential::graph<
    stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty, EdgeProperty,
    stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
    VertexProperty, EdgeProperty> >::edge_map<E>*>
make_stapl_edge_property_map(const stapl::sequential::graph<stapl::UNDIRECTED,
    stapl::MULTIEDGES, VertexProperty, EdgeProperty,
    stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
    VertexProperty, EdgeProperty> >::edge_map<E>& a)
{
  typedef stapl_edge_property_map<E,const E&,const stapl::sequential::graph<
      stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
          VertexProperty, EdgeProperty> >::edge_map<E>*> pmap_type;
  return pmap_type(&a);
}


template <class E>
stapl_edge_property_map<E, E&, stapl::sequential::graph<stapl::UNDIRECTED,
    stapl::MULTIEDGES, VertexProperty, EdgeProperty,
    stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty> >::edge_map<E>*>
make_stapl_edge_property_map(
    stapl::sequential::graph<stapl::UNDIRECTED, stapl::MULTIEDGES,
        VertexProperty, EdgeProperty, stapl::sequential::adj_graph_traits<
            stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty,
            EdgeProperty> >::edge_map<E>& a)
{
  typedef stapl_edge_property_map<E, E&, stapl::sequential::graph<
      stapl::UNDIRECTED, stapl::MULTIEDGES, VertexProperty, EdgeProperty,
      stapl::sequential::adj_graph_traits<stapl::UNDIRECTED, stapl::MULTIEDGES,
      VertexProperty, EdgeProperty> >::edge_map<E>*> pmap_type;
  return pmap_type(&a);
}
*/
} // namespace boost

#endif

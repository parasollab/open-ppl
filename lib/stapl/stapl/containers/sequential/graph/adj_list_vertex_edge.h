/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_ADJ_LIST_VERTEX_EDGE_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_ADJ_LIST_VERTEX_EDGE_HPP

#include <iostream>
#include <vector>
#include <algorithm>
#include "graph_util.h"
#include "graph_iterator.h"
#ifdef _STAPL
# include <stapl/utility/memory_size.hpp>
# include <stapl/runtime/serialization.hpp>
# include <tuple>
#endif

namespace stapl {

namespace sequential {

//to be used in friend declarations
template <typename Traits>
class adjacency_list_graph;

//////////////////////////////////////////////////////////////////////
/// @brief Vertex for adjacency list for the graph.
///
/// It contains a descriptor and an adjacency list of edges.
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam AdjList Type of the edgelist for storing edges.
//////////////////////////////////////////////////////////////////////
template <typename VD, typename AdjList>
class vertex_impl
  : private properties::no_property
{
private:
  template <typename Traits>
  friend class adjacency_list_graph;

public:
  typedef VD                      vertex_descriptor;
  typedef AdjList                 edgelist_type;
  typedef properties::no_property property_type;

  vertex_impl(void)
    : m_vd(),
      m_edgelist()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a vertex for the graph with the given descriptor
  /// and property.
  /// @param vd The descriptor of this vertex.
  /// @param p The property of this vertex (no_property).
  //////////////////////////////////////////////////////////////////////
  vertex_impl(vertex_descriptor const& vd, property_type const&)
      : m_vd(vd)
  {  }

  vertex_impl(vertex_descriptor const& vd)
    : m_vd(vd)
  {  }

  property_type& property(void) noexcept
  {
    return *this;
  }

  property_type const& property(void) const noexcept
  {
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the descriptor of this vertex.
  ///
  /// The vertex descriptor can't be changed after the vertex is created.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor const& descriptor(void) const noexcept
  {
    return m_vd;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the edge-list of this vertex.
  //////////////////////////////////////////////////////////////////////
  edgelist_type& edgelist(void) noexcept
  {
    return m_edgelist;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the edge-list of this vertex.
  //////////////////////////////////////////////////////////////////////
  void edgelist(edgelist_type const * const edgelist)
  {
    m_edgelist = *edgelist;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the edge-list of this vertex.
  //////////////////////////////////////////////////////////////////////
  edgelist_type const& edgelist(void) const noexcept
  {
    return m_edgelist;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the edge-list of this vertex.
  //////////////////////////////////////////////////////////////////////
  edgelist_type& edges(void) noexcept
  {
    return m_edgelist;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears the edge-list of this vertex.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    m_edgelist.clear();
  }

#ifdef _STAPL
  size_t memory_size(void) const noexcept
  {
    return sizeof(m_vd) + memory_used<edgelist_type>::size(m_edgelist);
  }

  void define_type(typer& t)
  {
    t.base<properties::no_property>(*this);
    t.member(m_vd);
    t.member(m_edgelist);
  }
#endif

protected:
  /// The descriptor of this vertex.
  VD m_vd;
  /// The edge-list of this vertex.
  edgelist_type m_edgelist;
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex for adjacency list for the graph with property.
///
/// It contains a descriptor and an adjacency list of edges, and extends
/// the @ref vertex_impl class with data/methods related to the property
/// field.
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam Property Property type for the vertex.
/// @tparam AdjList Type of the edgelist for storing edges.
//////////////////////////////////////////////////////////////////////
template <typename VD, typename Property, typename AdjList>
class vertex_property_impl
  : public vertex_impl<VD, AdjList>
{
private:
  typedef vertex_impl<VD, AdjList> base_type;

public:
  typedef VD       vertex_descriptor;
  typedef Property property_type;

  vertex_property_impl(void)
    : m_property()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a vertex for the graph with the given descriptor
  /// and property.
  /// @param vd The descriptor of this vertex.
  /// @param p The property of this vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_property_impl(vertex_descriptor const& vd, Property const& p)
    : base_type(vd),
      m_property(p)
  { }

  property_type& property(void) noexcept
  {
    return m_property;
  }

  property_type const& property(void) const noexcept
  {
    return m_property;
  }

#ifdef _STAPL
  size_t memory_size(void) const noexcept
  {
    return base_type::memory_size()
      + memory_used<property_type>::size(m_property);
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_property);
  }
#endif
protected:
  /// The property of this vertex.
  property_type m_property;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper for selecting the vertex implementation depending on
/// if the graph is with or without properties on vertices.
/// @tparam VD The type of the vertex descriptor.
/// @tparam Property The type of the vertex property.
/// @tparam AdjList The type of the adjacency list.
//////////////////////////////////////////////////////////////////////
template <typename VD, typename Property, typename AdjList>
struct select_vertex
{
  typedef vertex_property_impl<VD, Property, AdjList> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of helper @ref select_vertex for selecting
/// the vertex implementation with property storage of type @ref no_property
/// if the property is not specified.
/// @tparam VD The type of the vertex descriptor.
/// @tparam AdjList The type of the adjacency list.
//////////////////////////////////////////////////////////////////////
template <typename VD, typename AdjList>
struct select_vertex<VD, properties::no_property, AdjList>
{
  typedef vertex_impl<VD, AdjList> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief The Graph Edge without edge property storage.
///
/// If the graph uses @ref no_property for the edge property type,
/// then the graph edge contains just the @ref edge_descriptor_impl.
/// If the graph is with properties then the edge contains an
/// @ref edge_descriptor and the property.
//////////////////////////////////////////////////////////////////////
template <typename VertexDescriptor>
class graph_edge
{
public:
  typedef VertexDescriptor                            vertex_descriptor;
  typedef edge_descriptor_impl<vertex_descriptor>     edge_descriptor_type;
  typedef typename edge_descriptor_type::edge_id_type edge_id_type;
  typedef properties::no_property                     property_type;

  graph_edge(void)
    : m_ed()
  { }

  graph_edge(edge_descriptor_type const& ed)
    : m_ed(ed)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an edge for the graph with the given descriptor
  /// and property.
  /// @param ed The descriptor of this edge.
  /// @param p The property of this edge (no_property).
  //////////////////////////////////////////////////////////////////////
  graph_edge(edge_descriptor_type const& ed, property_type const&)
      : m_ed(ed)
  { }

  edge_descriptor_type descriptor(void) const noexcept
  {
    return m_ed;
  }

  edge_id_type id(void) const noexcept
  {
    return m_ed.id();
  }

  vertex_descriptor source(void) const noexcept
  {
    return m_ed.source();
  }

  vertex_descriptor target(void) const noexcept
  {
    return m_ed.target();
  }

  property_type property(void) const noexcept
  {
    return property_type{};
  }

#ifdef _STAPL
  typedef std::tuple<edge_descriptor_type> member_types;

  void define_type(typer& t)
  {
    t.member(m_ed);
  }
#endif

protected:
  /// The edge descriptor for this edge.
  edge_descriptor_type m_ed;

  //////////////////////////////////////////////////////////////////////
  /// @brief Reverses the edge descriptor of an edge.
  /// @param ed The edge.
  /// @return An edge with the same id as the input edge,
  /// but with source and target descriptors switched.
  //////////////////////////////////////////////////////////////////////
  friend graph_edge reverse(graph_edge const& other)
  {
    return graph_edge(reverse(other.m_ed));
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief The Graph Edge with edge property storage.
///
/// If the graph uses @ref no_property for the edge property type,
/// then the graph edge contains just the @ref edge_descriptor_impl.
/// If the graph is with properties then the edge contains an
/// @ref edge_descriptor and the property.
//////////////////////////////////////////////////////////////////////
template <typename VertexDescriptor, typename Property>
class graph_property_edge
{
public:
  typedef VertexDescriptor                            vertex_descriptor;
  typedef edge_descriptor_impl<vertex_descriptor>     edge_descriptor_type;
  typedef typename edge_descriptor_type::edge_id_type edge_id_type;
  typedef Property                                    property_type;
  typedef Property&                                   property_reference;
  typedef Property const&                             const_property_reference;

  graph_property_edge(void)
   : m_ed(),
     m_property()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an edge for the graph with the given descriptor
  /// and property.
  /// @param ed The descriptor of this edge.
  /// @param p The property of this edge.
  //////////////////////////////////////////////////////////////////////
  graph_property_edge(edge_descriptor_type const& ed, property_type const& p)
    : m_ed(ed),
      m_property(p)
  { }

  property_reference property(void) noexcept
  {
    return m_property;
  }

  const_property_reference property(void) const noexcept
  {
    return m_property;
  }

  edge_descriptor_type descriptor(void) const noexcept
  {
    return m_ed;
  }

  edge_id_type id(void) const noexcept
  {
    return m_ed.id();
  }

  vertex_descriptor source(void) const noexcept
  {
    return m_ed.source();
  }

  vertex_descriptor target(void) const noexcept
  {
    return m_ed.target();
  }

#ifdef _STAPL
  void define_type(typer& t)
  {
    t.member(m_ed);
    t.member(m_property);
  }
#endif

protected:
  /// The edge descriptor for this edge.
  edge_descriptor_type m_ed;
  /// The property of this edge.
  property_type m_property;

  //////////////////////////////////////////////////////////////////////
  /// @brief Reverses the edge descriptor of an edge.
  /// @param ed The edge.
  /// @return An edge with the same property and id as the input edge,
  /// but with source and target descriptors switched.
  //////////////////////////////////////////////////////////////////////
  friend graph_property_edge reverse(graph_property_edge const& other)
  {
    return graph_property_edge(reverse(other.m_ed), other.m_property);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A short graph edge with no property.
///
/// A short edge is one that only has information about the target,
/// and no source or id information.
//////////////////////////////////////////////////////////////////////
template<typename VertexDescriptor>
class short_graph_edge
{
public:
  using vertex_descriptor = VertexDescriptor;
  using edge_descriptor_type = edge_descriptor_impl<vertex_descriptor>;
  using edge_id_type = typename edge_descriptor_type::edge_id_type;
  using property_type = properties::no_property;

  short_graph_edge(void) = default;

  short_graph_edge(edge_descriptor_type const& ed)
      : m_target(ed.target())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an edge for the graph with the given descriptor
  /// and property.
  /// @param ed The descriptor of this edge.
  /// @param p The property of this edge (no_property).
  //////////////////////////////////////////////////////////////////////
  short_graph_edge(edge_descriptor_type const& ed, property_type const&)
      : m_target(ed.target())
  { }

  vertex_descriptor target(void) const noexcept
  {
    return m_target;
  }

  property_type property(void) const noexcept
  {
    return property_type{};
  }

#ifdef _STAPL
  typedef std::tuple<edge_descriptor_type> member_types;

  void define_type(typer& t)
  {
    t.member(m_target);
  }
#endif

protected:
  vertex_descriptor m_target;
};


//////////////////////////////////////////////////////////////////////
/// @brief A short graph edge with a property.
///
/// A short edge is one that only has information about the target,
/// and no source or id information.
//////////////////////////////////////////////////////////////////////
template<typename VertexDescriptor, typename Property>
class short_graph_property_edge
{
public:
  using vertex_descriptor = VertexDescriptor;
  using edge_descriptor_type = edge_descriptor_impl<vertex_descriptor>;
  using edge_id_type = typename edge_descriptor_type::edge_id_type;
  using property_type = Property;

  short_graph_property_edge(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an edge for the graph with the given descriptor
  /// and property.
  /// @param ed The descriptor of this edge.
  /// @param p The property of this edge (no_property).
  //////////////////////////////////////////////////////////////////////
  short_graph_property_edge(edge_descriptor_type const& ed,
                            property_type p = property_type{})
      : m_target(ed.target()), m_property(std::move(p))
  { }

  vertex_descriptor target(void) const noexcept
  {
    return m_target;
  }

  property_type property(void) const noexcept
  {
    return m_property;
  }

#ifdef _STAPL
  typedef std::tuple<edge_descriptor_type> member_types;

  void define_type(typer& t)
  {
    t.member(m_target);
    t.member(m_property);
  }
#endif

protected:
  vertex_descriptor m_target;
  property_type m_property;
};


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute the short edge type from a full edge edge
///
/// @tparam Edge The full edge
//////////////////////////////////////////////////////////////////////
template<typename Edge>
struct compute_short_edge_type
{
  static_assert(sizeof(Edge) == 0, "No transformation to short edge type");
};

template<typename VertexDescriptor>
struct compute_short_edge_type<graph_edge<VertexDescriptor>>
{
  using type = short_graph_edge<VertexDescriptor>;
};

template<typename VertexDescriptor, typename Property>
struct compute_short_edge_type<graph_property_edge<VertexDescriptor, Property>>
{
  using type = short_graph_property_edge<VertexDescriptor, Property>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper for selecting the edge implementation based on property
/// on edge and directedness of graph.
///
/// Other specializations for undirected are in undirected_util.h
/// @tparam VertexDescriptor The type of the vertex descriptor.
/// @tparam Property The type of the edge property.
/// @tparam Directedness Tag for directed or undirected graph.
//////////////////////////////////////////////////////////////////////
template <typename VertexDescriptor,
          typename Property,
          graph_attributes Directedness>
struct select_edge
{
  typedef graph_property_edge<VertexDescriptor, Property> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper for selecting the edge implementation based on property
/// on edge and directedness of graph. Specialized for when the graph is
/// directed and the edge property is @ref no_property.
///
/// Other specializations for undirected are in undirected_util.h
/// @tparam VertexDescriptor The type of the vertex descriptor.
//////////////////////////////////////////////////////////////////////
template <typename VertexDescriptor>
struct select_edge<VertexDescriptor, properties::no_property, DIRECTED>
{
  typedef graph_edge<VertexDescriptor> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Adjacency list of a vertex.
///
/// Implemented as a vector of edges
/// @tparam Edge The type of the edge.
//////////////////////////////////////////////////////////////////////
template <typename Edge>
class adjacency_descriptor_impl
{
  template <typename Traits> friend class adjacency_list_graph;
public:
  //infer property
  typedef Edge    edge_type;
  typedef typename select_adj_edge_iterator<
            typename std::vector<Edge>::iterator, Edge
          >::type iterator;
  typedef typename select_const_adj_edge_iterator<
            typename std::vector<Edge>::const_iterator, Edge
          >::type const_iterator;

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an adjacency list with @p n edges.
  //////////////////////////////////////////////////////////////////////
  adjacency_descriptor_impl(size_t n = 0)
    : m_data(n)
  { }

  template <typename Comp>
  void sort_edges(Comp comp)
  {
    std::sort(m_data.begin(), m_data.end(), comp);
  }

  iterator begin(void) noexcept
  {
    return iterator(m_data.begin());
  }

  const_iterator begin(void) const noexcept
  {
    return const_iterator(m_data.begin());
  }

  iterator end(void) noexcept
  {
    return iterator(m_data.end());
  }

  const_iterator end(void) const noexcept
  {
    return const_iterator(m_data.end());
  }

  size_t size(void) const noexcept
  {
    return m_data.size();
  }

#ifdef _STAPL
  void define_type(typer& t)
  {
    t.member(m_data);
  }
#endif

  void clear(void)
  {
    m_data.clear();
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Add the specified edge to the adjacency list.
  ///
  /// Interface specific to adjacency list -- accessible only from
  /// @ref adjacency_list_graph.
  /// @param ed The edge to be added.
  //////////////////////////////////////////////////////////////////////
  void add(Edge const& ed)
  {
    m_data.push_back(ed);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Erase the specified edge from the adjacency list.
  /// @param it An iterator to the edge to be erased.
  //////////////////////////////////////////////////////////////////////
  void erase(iterator it)
  {
    m_data.erase(it.base());
  }

  /// List of edges stored in an std::vector.
  std::vector<Edge> m_data;
};

} //end namespace sequential

} //end namespace stapl

#endif

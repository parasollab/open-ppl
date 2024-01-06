/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_UNDIRECTED_UTIL_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_UNDIRECTED_UTIL_HPP

#include "adj_list_vertex_edge.h"

namespace stapl {
namespace sequential {

//////////////////////////////////////////////////////////////////////
/// @brief The Graph Edge with edge property storage for an undirected graph.
/// @ingroup graphBaseUtil
///
/// If the graph uses @ref no_property for the edge property type,
/// then the graph edge contains just the @ref edge_descriptor_impl.
/// If the graph is with properties then the edge contains an
/// @ref edge_descriptor_impl and the property.
/// The property of the edge for undirected graphs is stored as a
/// pointer and shared between two sister edges, because edges
/// will always be added in pairs and they will have to have the same
/// property.
//////////////////////////////////////////////////////////////////////
template <class VertexDescriptor, class Property>
class undirected_graph_edge_property
{
  typedef undirected_graph_edge_property<VertexDescriptor, Property> this_type;

public:
  typedef VertexDescriptor vertex_descriptor;
  typedef edge_descriptor_impl<vertex_descriptor> edge_descriptor_type;
  typedef typename edge_descriptor_type::edge_id_type edge_id_type;
  typedef Property property_type;
  typedef Property& property_reference;

  undirected_graph_edge_property()
      : m_property(NULL)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an edge for the graph with the given descriptor
  /// and property.
  /// @param ed The descriptor of this edge.
  /// @param p The property of this edge.
  //////////////////////////////////////////////////////////////////////
  undirected_graph_edge_property(edge_descriptor_type ed,
                                 property_type const& p)
      : m_ed(ed)
  {
    m_property = new property_type(p);
  }

  property_type& property()
  {
    return *m_property;
  }

  property_type const& property() const
  {
    return *m_property;
  }

  edge_descriptor_type descriptor() const
  {
    return m_ed;
  }

  edge_id_type id() const
  {
    return m_ed.id();
  }

  vertex_descriptor source() const
  {
    return m_ed.source();
  }

  vertex_descriptor target() const
  {
    return m_ed.target();
  }

  void display()
  {
    std::cout << m_ed.source() << "->" << m_ed.target() << "@" << &(*m_property)
        << "] ";
  }

#ifdef _STAPL
  void define_type(stapl::typer& t)
  {
    t.member(m_ed);
    t.member(m_property);
  }
#endif

protected:
  /// The edge descriptor for this edge.
  edge_descriptor_type m_ed;
  /// The property of this edge.
  property_type* m_property;

  //////////////////////////////////////////////////////////////////////
  /// @brief Reverses the edge descriptor of an edge.
  /// @param other The edge to reverse.
  /// @return An edge with the same property and id as the input edge,
  /// but with source and target descriptors switched.
  //////////////////////////////////////////////////////////////////////
  friend this_type reverse(this_type const& other)
  {
    this_type new_edge = other;
    new_edge.m_ed = reverse(other.m_ed);
    return new_edge;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes the property of this edge.
  //////////////////////////////////////////////////////////////////////
  void clear()
  {
    delete this->m_property;
  }

};


//////////////////////////////////////////////////////////////////////
/// @brief Helper class for undirected graph.
/// @ingroup graphBaseUtil
///
/// Edge descriptors are created with a special flag, and edge properties
/// are cleared differently than for directed graph
//////////////////////////////////////////////////////////////////////
template <class EdgeDescriptor, class Iterator, class Property>
struct undirected_edge_helper
{
  static void clear(Iterator it)
  {
    delete &((*it).property());
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper class for undirected graph.
/// @ingroup graphBaseUtil
///
/// Edge descriptors are created with a special flag, and edge properties
/// are cleared differently than for directed graph
//////////////////////////////////////////////////////////////////////
template <class EdgeDescriptor, class Iterator>
struct undirected_edge_helper<EdgeDescriptor, Iterator, properties::no_property>
{
  static void clear(Iterator)
  {
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper for selecting the edge implementation based on property
/// on edge and directedness of graph, specialized for undirected graph.
/// @ingroup graphBaseUtil
///
/// Other specializations for directed are in adj_list_vertex_edge.h
/// @tparam VertexDescriptor The type of the vertex descriptor.
/// @tparam Property The type of the edge property.
//////////////////////////////////////////////////////////////////////
template <class VertexDescriptor, class Property>
struct select_edge<VertexDescriptor, Property, UNDIRECTED>
{
  typedef undirected_graph_edge_property<VertexDescriptor, Property> type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper for selecting the edge implementation based on property
/// on edge and directedness of graph, specialized for undirected graph.
/// @ingroup graphBaseUtil
///
/// Other specializations for directed are in adj_list_vertex_edge.h
/// @tparam VertexDescriptor The type of the vertex descriptor.
//////////////////////////////////////////////////////////////////////
template <class VertexDescriptor>
struct select_edge<VertexDescriptor, properties::no_property, UNDIRECTED>
{
  typedef graph_edge<VertexDescriptor> type;
};

template<typename VD, typename Property>
struct compute_short_edge_type<undirected_graph_edge_property<VD, Property>>
{
  using type = undirected_graph_edge_property<VD, Property>;
};

} // namespace sequential
} // namespace stapl

#endif

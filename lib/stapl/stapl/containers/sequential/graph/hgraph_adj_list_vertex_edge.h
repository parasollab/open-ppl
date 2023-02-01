/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_HGRAPH_ADJ_LIST_VERTEX_EDGE_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_HGRAPH_ADJ_LIST_VERTEX_EDGE_HPP

#include <stapl/containers/sequential/graph/adj_list_vertex_edge.h>
#include <stapl/domains/interval.hpp>

namespace stapl {
namespace sequential {

//to be used in friend declarations
template <class Traits> class adjacency_list_graph;

//////////////////////////////////////////////////////////////////////
/// @brief Vertex for adjacency list for the @ref hierarchical_graph.
/// @ingroup graphBaseStorage
///
/// It contains a descriptor and an adjacency list of edges, as well
/// as a set of children vertices for this graph.
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam AdjList Type of the edgelist for storing edges.
//////////////////////////////////////////////////////////////////////
template <class VD, class AdjList>
class hgraph_vertex_impl
  : public vertex_impl<VD, AdjList>
{
  typedef hgraph_vertex_impl<VD, AdjList> this_type;
  typedef vertex_impl<VD, AdjList> base_type;
  template <class Traits> friend class adjacency_list_graph;

public:
  typedef VD vertex_descriptor;
  typedef properties::no_property property_type;
  typedef domset1D<VD> child_set_type;

protected:
  child_set_type m_children; //children for hierarchical graph

public:
  hgraph_vertex_impl()
      : base_type()
  {
  }

  hgraph_vertex_impl(const this_type& other)
      : base_type(other), m_children(other.m_children)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a vertex for the graph with the given descriptor
  /// and property.
  /// @param vd The descriptor of this vertex.
  /// @param p The property of this vertex (no_property).
  //////////////////////////////////////////////////////////////////////
  hgraph_vertex_impl(vertex_descriptor const& vd, property_type const& np)
      : base_type(vd, np)
  {
  }

  child_set_type children() const
  {
    return m_children;
  }

  void set_child(child_set_type const& children)
  {
    m_children = children;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a child to this vertex.
  /// @param child The descriptor of the new child.
  //////////////////////////////////////////////////////////////////////
  void add_child(vertex_descriptor const& child)
  {
    m_children += child;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a set of children to this vertex.
  /// @param children The domain of the new children.
  //////////////////////////////////////////////////////////////////////
  void add_child(child_set_type const& children)
  {
    m_children += children;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes a child from this vertex.
  /// @param child The descriptor of the child.
  //////////////////////////////////////////////////////////////////////
  void delete_child(vertex_descriptor const& child)
  {
    m_children -= child;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Deletes a set of children from this vertex.
  /// @param children The domain of the children.
  //////////////////////////////////////////////////////////////////////
  void delete_child(child_set_type const& children)
  {
    m_children -= children;
  }

#ifdef _STAPL
  size_t memory_size(void) const
  {
    return base_type::memory_size() +
    memory_used<child_set_type>::size(m_children);
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_children);
  }
#endif
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex for adjacency list for the @ref hierarchical_graph
/// with property.
/// @ingroup graphBaseStorage
///
/// It contains a descriptor and an adjacency list of edges, and extends
/// the @ref hgraph_vertex_impl class with data/methods related to the
/// property field.
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam Property Property type for the vertex.
/// @tparam AdjList Type of the edgelist for storing edges.
//////////////////////////////////////////////////////////////////////
template <class VD, class Property, class AdjList>
class hgraph_vertex_property_impl
  : public hgraph_vertex_impl<VD, AdjList>
{
  typedef hgraph_vertex_property_impl<VD, Property, AdjList> this_type;
  typedef hgraph_vertex_impl<VD, AdjList> base_type;
public:
  typedef VD vertex_descriptor;
  typedef Property property_type;

protected:
  property_type m_property;

public:
  hgraph_vertex_property_impl()
      : base_type()
  {
  }

  hgraph_vertex_property_impl(this_type const& other)
      : base_type(other), m_property(other.m_property)
  {
  }

  hgraph_vertex_property_impl(vertex_descriptor const& vd, Property const& p)
    : base_type(vd, properties::no_property()), m_property(p)
  {
  }

  property_type& property()
  {
    return m_property;
  }

  property_type const& property() const
  {
    return m_property;
  }

#ifdef _STAPL
  size_t memory_size(void) const
  {
    return base_type::memory_size() +
    memory_used<property_type>::size(m_property);
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_property);
  }
#endif

};


//////////////////////////////////////////////////////////////////////
/// @brief Helper for selecting the vertex implementation depending on
/// if the hierarchical graph is with or without properties on vertices
/// @ingroup graphBaseUtil
/// @tparam VD The type of the vertex descriptor.
/// @tparam AdjList The type of the adjacency list.
/// @tparam Property The type of the vertex property.
//////////////////////////////////////////////////////////////////////
template <class VD, class AdjList, class Property>
struct hgraph_select_vertex
{
  typedef hgraph_vertex_property_impl<VD, AdjList, Property> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of helper @ref hgraph_select_vertex for selecting
/// the vertex implementation with property storage of type @ref no_property
/// if the property is not specified.
/// @ingroup graphBaseUtil
/// @tparam VD The type of the vertex descriptor.
/// @tparam AdjList The type of the adjacency list.
//////////////////////////////////////////////////////////////////////
template <class VD, class AdjList>
struct hgraph_select_vertex<VD, AdjList, properties::no_property>
{
  typedef hgraph_vertex_impl<VD, AdjList> type;
};

} // namespace sequential
} // namespace stapl

#endif

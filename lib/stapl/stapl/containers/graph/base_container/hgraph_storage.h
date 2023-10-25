/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_HGRAPH_STORAGE_H
#define STAPL_CONTAINERS_GRAPH_HGRAPH_STORAGE_H

#include <stapl/containers/graph/base_container/graph_storage.h>
#include <stapl/containers/sequential/graph/hgraph_adj_list_vertex_edge.h>

namespace stapl {

/// To be used in friend declarations
template<class Traits> class adjacency_list_model;


//////////////////////////////////////////////////////////////////////
/// @brief Vertex for adjacency list hierarchical graph.
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam Property Property type of the vertex.
/// @tparam AdjList Type of the edgelist for storing edges.
/// @ingroup pgraphAdjacency
//////////////////////////////////////////////////////////////////////
template<class VD, class Property, class AdjList>
class hgraph_vertex_adj_list_impl
  : public sequential::hgraph_select_vertex<VD,Property,AdjList>::type
{
  template<class Traits> friend class adjacency_list_model;
  typedef typename sequential::hgraph_select_vertex
          <VD,Property,AdjList>::type            base_type;
 public:
  typedef VD                                     vertex_descriptor;
  typedef AdjList                                edgelist_type;
  typedef AdjList                                adj_edges_type;
  typedef typename AdjList::edge_descriptor      edge_descriptor;
  typedef typename AdjList::iterator             edgelist_it;
  typedef typename AdjList::const_iterator       const_edgelist_it;
  typedef edgelist_it                            adj_edge_iterator;
  typedef const_edgelist_it                      const_adj_edge_iterator;
  typedef Property                               property_type;
  typedef Property&                              property_reference;
  typedef Property const&                        const_property_reference;

  hgraph_vertex_adj_list_impl()
    : base_type()
  { }

  hgraph_vertex_adj_list_impl(hgraph_vertex_adj_list_impl const& other)
    : base_type(other)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a vertex for the hierarchical graph with the given
  /// descriptor and property.
  /// @param vd The descriptor of this vertex.
  /// @param p The property of this vertex.
  //////////////////////////////////////////////////////////////////////
  hgraph_vertex_adj_list_impl(vertex_descriptor vd, Property const& p)
    : base_type(vd, p)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the beginning of the edgelist of this vertex.
  //////////////////////////////////////////////////////////////////////
  adj_edge_iterator begin()
  { return this->m_edgelist.begin(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the end of the edgelist of this vertex.
  //////////////////////////////////////////////////////////////////////
  adj_edge_iterator end()
  { return this->m_edgelist.end(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the beginning of the edgelist of this vertex.
  //////////////////////////////////////////////////////////////////////
  const_adj_edge_iterator begin() const
  { return this->m_edgelist.begin(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the end of the edgelist of this vertex.
  //////////////////////////////////////////////////////////////////////
  const_adj_edge_iterator end() const
  { return this->m_edgelist.end(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the edge with the specified target.
  /// @param vd The descriptor of the target vertex.
  //////////////////////////////////////////////////////////////////////
  adj_edge_iterator find_edge(vertex_descriptor const& vd)
  { return this->m_edgelist.find(vd); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the specified edge.
  /// @param ed The descriptor of the edge.
  //////////////////////////////////////////////////////////////////////
  adj_edge_iterator find_edge(edge_descriptor const& ed)
  { return this->m_edgelist.find(ed); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of outgoing edges of this vertex.
  //////////////////////////////////////////////////////////////////////
  size_t size() const
  { return this->m_edgelist.size(); }

  size_t memory_size(void) const
  { return base_type::memory_size(); }

#ifdef _STAPL
  void define_type(typer& t)
  { t.base<base_type>(*this); }
#endif
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref hgraph_vertex_adj_list_impl.
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam Property Property type of the vertex.
/// @tparam AdjList Type of the edgelist for storing edges.
/// @tparam Accessor Type of the accessor for the proxy.
/// @ingroup pgraphAdjacency
//////////////////////////////////////////////////////////////////////
template<class VD, class Property, class AdjList, typename Accessor>
class proxy<hgraph_vertex_adj_list_impl<VD, Property, AdjList>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef hgraph_vertex_adj_list_impl<VD, Property, AdjList> target_t;
  typedef typename target_t::adj_edge_iterator       adj_edge_iter_t;
  typedef typename target_t::const_adj_edge_iterator const_adj_edge_iter_t;

public:
  typedef typename target_t::property_type            property_type;
  typedef typename Accessor::property_reference       property_reference;
  typedef typename Accessor::const_property_reference const_property_reference;
  typedef typename target_t::vertex_descriptor        vertex_descriptor;
  typedef typename target_t::edge_descriptor          edge_descriptor;
  typedef typename target_t::adj_edges_type           adj_edges_type;
  typedef typename target_t::child_set_type           child_set_type;

  typedef member_iterator<adj_edge_iter_t, Accessor>  adj_edge_iterator;
  typedef member_iterator<const_adj_edge_iter_t,
                          Accessor>                   const_adj_edge_iterator;

  explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  proxy const& operator=(proxy const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs)
  {
    Accessor::write(rhs);
    return *this;
  }

  operator target_t() const
  { return Accessor::read(); }

  vertex_descriptor descriptor()  const
  { return Accessor::ref().descriptor(); }

  property_reference property()
  { return Accessor::property(); }

  property_reference property() const
  { return Accessor::property(); }

  const_property_reference const_property() const
  { return Accessor::const_property(); }

  adj_edge_iterator begin()
  { return adj_edge_iterator(Accessor::invoke(&target_t::begin), *this); }

  adj_edge_iterator end()
  { return adj_edge_iterator(Accessor::invoke(&target_t::end), *this); }

  adj_edge_iterator find_edge(vertex_descriptor const& vd)
  { return adj_edge_iterator(Accessor::invoke(&target_t::find, vd), *this); }

  adj_edge_iterator find_edge(edge_descriptor const& ed)
  { return adj_edge_iterator(Accessor::invoke(&target_t::find, ed), *this); }

  const_adj_edge_iterator begin() const
  {
    return const_adj_edge_iterator(Accessor::const_invoke(&target_t::begin),
                                   *this);
  }

  const_adj_edge_iterator end() const
  {
    return const_adj_edge_iterator(Accessor::const_invoke(&target_t::end),
                                   *this);
  }


  const_adj_edge_iterator find_edge(vertex_descriptor const& vd) const
  {
    return const_adj_edge_iterator(Accessor::const_invoke(&target_t::find, vd),
                                   *this);
  }

  const_adj_edge_iterator find_edge(edge_descriptor const& ed) const
  {
    return const_adj_edge_iterator(Accessor::const_invoke(&target_t::find, ed),
                                   *this);
  }

  size_t size(void) const
  { return Accessor::ref().size(); }

  adj_edges_type edges() const
  { return Accessor::ref().edges();}

  child_set_type children() const
  {
    return Accessor::const_invoke(&target_t::children);
  }

  void set_child(child_set_type const& children)
  {
    Accessor::invoke(&target_t::set_child, children);
  }

  void add_child(vertex_descriptor const& child)
  {
    void (target_t::*fct_ptr)(vertex_descriptor const&) =
        &target_t::add_child;
    Accessor::invoke(fct_ptr, child);
  }

  void add_child(child_set_type const& children)
  {
    void (target_t::*fct_ptr)(child_set_type const&) = &target_t::add_child;
    Accessor::invoke(fct_ptr, children);
  }

  void delete_child(vertex_descriptor const& child)
  {
    void (target_t::*fct_ptr)(vertex_descriptor const&) =
        &target_t::delete_child;
    Accessor::invoke(fct_ptr, child);
  }

  void delete_child(child_set_type const& children)
  {
    void (target_t::*fct_ptr)(child_set_type const&) =
        &target_t::delete_child;
    Accessor::invoke(fct_ptr, children);
  }
}; // struct proxy


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref hgraph_vertex_adj_list_impl.
/// Specialized for @ref local_accessor_graph, which lets us use a
/// faster iterator over the edges than the @ref member_iterator
/// needed for the @ref container_accessor.
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam Property Property type of the vertex.
/// @tparam AdjList Type of the edgelist for storing edges.
/// @ingroup pgraphAdjacency
//////////////////////////////////////////////////////////////////////
template<class VD, class Property, class AdjList, typename Container>
class proxy<hgraph_vertex_adj_list_impl<VD, Property, AdjList>,
            local_accessor_graph<Container> >
  : public local_accessor_graph<Container>
{
private:
  typedef local_accessor_graph<Container> accessor_t;
  friend class proxy_core_access;
  typedef hgraph_vertex_adj_list_impl<VD, Property, AdjList> target_t;

public:
  typedef typename target_t::property_type              property_type;
  typedef typename accessor_t::property_reference       property_reference;
  typedef typename accessor_t::const_property_reference
                                                       const_property_reference;
  typedef typename target_t::vertex_descriptor          vertex_descriptor;
  typedef typename target_t::edge_descriptor            edge_descriptor;
  typedef typename target_t::adj_edge_iterator          adj_edge_iterator;
  typedef typename target_t::const_adj_edge_iterator    const_adj_edge_iterator;
  typedef typename target_t::adj_edges_type             adj_edges_type;
  typedef typename target_t::child_set_type             child_set_type;

  explicit proxy(accessor_t const& acc)
    : accessor_t(acc)
  { }

  proxy const& operator=(proxy const& rhs)
  {
    accessor_t::write(rhs);
    return *this;
  }

  proxy const& operator=(target_t const& rhs)
  {
    accessor_t::write(rhs);
    return *this;
  }

  operator target_t() const
  { return accessor_t::read(); }

  vertex_descriptor descriptor()  const
  { return accessor_t::ref().descriptor(); }

  property_reference property()
  { return accessor_t::property(); }

  property_reference property() const
  { return accessor_t::property(); }

  const_property_reference const_property() const
  { return accessor_t::const_property(); }

  adj_edge_iterator begin()
  { return accessor_t::ref().begin(); }

  adj_edge_iterator end()
  { return accessor_t::ref().end(); }

  adj_edge_iterator find_edge(vertex_descriptor const& vd)
  { return accessor_t::ref().find(vd); }

  adj_edge_iterator find_edge(edge_descriptor const& ed)
  { return accessor_t::ref().find(ed); }

  const_adj_edge_iterator begin() const
  { return accessor_t::ref().begin(); }

  const_adj_edge_iterator end() const
  { return accessor_t::ref().end(); }

  const_adj_edge_iterator find_edge(vertex_descriptor const& vd) const
  { return accessor_t::ref().find(vd); }

  const_adj_edge_iterator find_edge(edge_descriptor const& ed) const
  { return accessor_t::ref().find(ed); }

  size_t size(void) const
  { return accessor_t::ref().size(); }

  adj_edges_type edges() const
  { return accessor_t::ref().edges();}

  child_set_type children() const
  {
    return accessor_t::const_invoke(&target_t::children);
  }

  void set_child(child_set_type const& children)
  {
    accessor_t::invoke(&target_t::set_child, children);
  }

  void add_child(vertex_descriptor const& child)
  {
    void (target_t::*fct_ptr)(vertex_descriptor const&) =
        &target_t::add_child;
    accessor_t::invoke(fct_ptr, child);
  }

  void add_child(child_set_type const& children)
  {
    void (target_t::*fct_ptr)(child_set_type const&) = &target_t::add_child;
    accessor_t::invoke(fct_ptr, children);
  }

  void delete_child(vertex_descriptor const& child)
  {
    void (target_t::*fct_ptr)(vertex_descriptor const&) =
        &target_t::delete_child;
    accessor_t::invoke(fct_ptr, child);
  }

  void delete_child(child_set_type const& children)
  {
    void (target_t::*fct_ptr)(child_set_type const&) =
        &target_t::delete_child;
    accessor_t::invoke(fct_ptr, children);
  }
}; // struct proxy



//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref memory_used for
/// @ref hgraph_vertex_adj_list_impl.
/// @tparam VD Vertex descriptor type for the graph.
/// @tparam Property Property type of the vertex.
/// @tparam AdjList Type of the edgelist for storing edges.
/// @ingroup pgraphAdjacency
//////////////////////////////////////////////////////////////////////
template<class VD, class Property, class AdjList>
struct memory_used<hgraph_vertex_adj_list_impl<VD,Property,AdjList> >
{
  typedef hgraph_vertex_adj_list_impl<VD,Property,AdjList> vertex_type;
  static size_t size(vertex_type const& v)
  { return v.memory_size(); }
};

} // namespace stapl
#endif

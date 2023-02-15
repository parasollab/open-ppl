/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_VIEW_PROPERTY_ADAPTOR_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_VIEW_PROPERTY_ADAPTOR_HPP

#include <iterator>
#include <boost/iterator/iterator_adaptor.hpp>
#include <stapl/utility/use_default.hpp>

//////////////////////////////////////////////////////////////////////
/// @file graph_view_property_adaptor.h
/// This file contains specializations for views with the same domain,
/// edge set, edge property, etc. as the input Graph but with a
/// custom vertex property.
//////////////////////////////////////////////////////////////////////

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialized iterator with property adaptor to filter
/// particular members of the property.
/// @ingroup graphBaseUtil
///
/// The property adaptor used in this iterator is default constructed.
//////////////////////////////////////////////////////////////////////
template <typename VertexIterator, class EdgeIter, class PropertyAdaptor>
class va_iterator
  : public boost::iterator_adaptor<
      va_iterator<VertexIterator, EdgeIter, PropertyAdaptor>, VertexIterator>
{
private:
  typedef boost::iterator_adaptor<
      va_iterator<VertexIterator, EdgeIter, PropertyAdaptor>,
    VertexIterator> base_type;
  PropertyAdaptor m_adapt;
public:
  typedef typename VertexIterator::vertex_descriptor vertex_descriptor;
  typedef typename PropertyAdaptor::result_type value_type;
  typedef typename PropertyAdaptor::result_type property_type;

  va_iterator() = default;

  va_iterator(VertexIterator iterator)
      : base_type(iterator)
  {
  }

  property_type operator*()
  {
    return m_adapt(this->base_reference());
  }

  vertex_descriptor descriptor() const
  {
    return this->base_reference().descriptor();
  }

  property_type property()
  {
    return m_adapt(this->base_reference());
  }

  EdgeIter begin()
  {
    return EdgeIter(this->base_reference().begin());
  }

  EdgeIter end()
  {
    return EdgeIter(this->base_reference().end());
  }

  size_t size(void) const
  {
    return this->base_reference().size();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialized iterator with property adaptor to filter
/// particular members of the property (const).
/// @ingroup graphBaseUtil
///
/// The property adaptor used in this iterator is default constructed.
//////////////////////////////////////////////////////////////////////
template <typename VertexIterator, class EdgeIter, class PropertyAdaptor>
class const_va_iterator
  : public boost::iterator_adaptor<const_va_iterator<VertexIterator, EdgeIter,
                                                     PropertyAdaptor>,
                                   VertexIterator>
{
private:
  typedef boost::iterator_adaptor<
      const_va_iterator<VertexIterator, EdgeIter, PropertyAdaptor>,
      VertexIterator> base_type;
  PropertyAdaptor m_adapt;
public:

  typedef typename VertexIterator::vertex_descriptor vertex_descriptor;
  typedef typename PropertyAdaptor::result_type value_type;
  typedef typename PropertyAdaptor::result_type property_type;

  const_va_iterator() = default;

  const_va_iterator(VertexIterator iterator)
      : base_type(iterator)
  {
  }

  //iterator and const_iterator interoperability
  template <typename Iter, typename Eiter>
  inline const_va_iterator(
      const va_iterator<Iter, Eiter, PropertyAdaptor>& other)
      : base_type(other.base())
  {
  }

  property_type operator*() const
  {
    return m_adapt(this->base_reference());
  }

  vertex_descriptor descriptor() const
  {
    return this->base_reference().descriptor();
  }

  property_type property() const
  {
    return m_adapt(this->base_reference());
  }

  EdgeIter begin()
  {
    return EdgeIter(this->base_reference().begin());
  }

  EdgeIter end()
  {
    return EdgeIter(this->base_reference().end());
  }

  size_t size(void) const
  {
    return this->base_reference().size();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialized vertex iterator for when only the edge is adapted./
/// @ingroup graphBaseUtil
///
/// The property adaptor used in this iterator is default constructed.
/////////////////////////////////////////////////////////////////////
template <typename VertexIterator, class EdgeIter>
class va_iterator<VertexIterator, EdgeIter, use_default>
  : public boost::iterator_adaptor<va_iterator<VertexIterator, EdgeIter,
                                               use_default>,
                                   VertexIterator>
{
private:
  typedef boost::iterator_adaptor<
      va_iterator<VertexIterator, EdgeIter, use_default>,
    VertexIterator> base_type;
public:
  typedef typename VertexIterator::vertex_descriptor vertex_descriptor;
  typedef typename VertexIterator::value_type value_type;
  typedef typename VertexIterator::property_type property_type;

  va_iterator() = default;

  va_iterator(VertexIterator iterator)
      : base_type(iterator)
  {
  }

  vertex_descriptor descriptor() const
  {
    return this->base_reference().descriptor();
  }

  property_type& property()
  {
    return this->base_reference().property();
  }

  EdgeIter begin()
  {
    return EdgeIter(this->base_reference().begin());
  }

  EdgeIter end()
  {
    return EdgeIter(this->base_reference().end());
  }

  size_t size(void) const
  {
    return this->base_reference().size();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialized vertex iterator for when only the edge is
/// adapted (const).
/// @ingroup graphBaseUtil
///
/// The property adaptor used in this iterator is default constructed.
//////////////////////////////////////////////////////////////////////
template <typename VertexIterator, class EdgeIter>
class const_va_iterator<VertexIterator, EdgeIter, use_default>
  : public boost::iterator_adaptor<
  const_va_iterator<VertexIterator, EdgeIter, use_default>, VertexIterator>
{
private:
  typedef boost::iterator_adaptor<
      const_va_iterator<VertexIterator, EdgeIter, use_default>,
    VertexIterator> base_type;
public:
  typedef typename VertexIterator::vertex_descriptor vertex_descriptor;
  typedef typename VertexIterator::value_type value_type;
  typedef typename VertexIterator::property_type property_type;

  const_va_iterator(void) = default;

  const_va_iterator(VertexIterator it)
    : base_type(it)
  { }

  //iterator and const_iterator interoperability
  template <typename _Iter, typename _Eiter>
  inline const_va_iterator(
      const va_iterator<_Iter, _Eiter, use_default>& _other)
      : base_type(_other.base())
  {
  }

  vertex_descriptor descriptor() const
  {
    return this->base_reference().descriptor();
  }

  property_type const& property() const
  {
    return this->base_reference().property();
  }

  EdgeIter begin()
  {
    return EdgeIter(this->base_reference().begin());
  }

  EdgeIter end()
  {
    return EdgeIter(this->base_reference().end());
  }

  size_t size(void) const
  {
    return this->base_reference().size();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Edge Iterator Adaptor for filtering out specific edge properties.
/// @ingroup graphBaseUtil
///
/// The property adaptor used in this iterator is default constructed.
//////////////////////////////////////////////////////////////////////
template <typename EdgeIterator, class PropertyAdaptor>
class ea_iterator
  : public boost::iterator_adaptor<
    ea_iterator<EdgeIterator, PropertyAdaptor>, EdgeIterator>
{
private:
  typedef boost::iterator_adaptor<ea_iterator<EdgeIterator, PropertyAdaptor>,
      EdgeIterator> base_type;
  PropertyAdaptor m_adapt;
public:
  typedef typename PropertyAdaptor::result_type value_type;
  typedef typename EdgeIterator::vertex_descriptor vertex_descriptor;
  ea_iterator() = default;

  ea_iterator(EdgeIterator iterator)
      : base_type(iterator)
  {
  }

  value_type operator*()
  {
    return m_adapt(this->base_reference());
  }

  vertex_descriptor source() const
  {
    return this->base_reference().source();
  }

  vertex_descriptor target() const
  {
    return this->base_reference().target();
  }

  size_t id() const
  {
    return this->base_reference().id();
  }

  value_type property()
  {
    return m_adapt(this->base_reference());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Edge Iterator Adaptor for filtering out specific edge
/// properties (const).
/// @ingroup graphBaseUtil
///
/// The property adaptor used in this iterator is default constructed.
//////////////////////////////////////////////////////////////////////
template <typename EdgeIterator, class PropertyAdaptor>
class const_ea_iterator
  : public boost::iterator_adaptor<
    ea_iterator<EdgeIterator, PropertyAdaptor>, EdgeIterator>
{
private:
  typedef boost::iterator_adaptor<ea_iterator<EdgeIterator, PropertyAdaptor>,
      EdgeIterator> base_type;
  PropertyAdaptor m_adapt;
public:
  typedef typename PropertyAdaptor::result_type value_type;
  typedef typename EdgeIterator::vertex_descriptor vertex_descriptor;

  const_ea_iterator() = default;

  const_ea_iterator(EdgeIterator iterator)
      : base_type(iterator)
  {
  }

  template <typename _Iter, typename _PA>
  inline const_ea_iterator(const ea_iterator<_Iter, _PA>& _other)
    : base_type(_other.base())
  {
  }

  value_type operator*() const
  {
    return m_adapt(this->base_reference());
  }

  vertex_descriptor source() const
  {
    return this->base_reference().source();
  }

  vertex_descriptor target() const
  {
    return this->base_reference().target();
  }

  size_t id() const
  {
    return this->base_reference().id();
  }

  value_type property() const
  {
    return m_adapt(this->base_reference());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Selector to infer adjacent edge iterator/property.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class VGraph, class EAdaptor>
struct internal_adj_edge_iterator_selector
{
  typedef ea_iterator<typename VGraph::adj_edge_iterator,
                      EAdaptor> adj_edge_iterator;
  typedef const_ea_iterator<typename VGraph::adj_edge_iterator,
                            EAdaptor> const_adj_edge_iterator;
  typedef typename VGraph::edge_descriptor edge_descriptor;
  typedef typename EAdaptor::result_type edge_property;
};

//////////////////////////////////////////////////////////////////////
/// @brief Selector to infer adjacent edge iterator/property.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class VGraph>
struct internal_adj_edge_iterator_selector<VGraph, use_default>
{
  typedef typename VGraph::adj_edge_iterator adj_edge_iterator;
  typedef typename VGraph::const_adj_edge_iterator const_adj_edge_iterator;
  typedef typename VGraph::edge_descriptor edge_descriptor;
  typedef typename VGraph::edge_property edge_property;
};

//////////////////////////////////////////////////////////////////////
/// @brief Selector to infer vertex iterator/property.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class VGraph, class EAdaptor, class VAdaptor>
struct internal_vertex_iterator_selector
{
  typedef typename internal_adj_edge_iterator_selector<VGraph,
    EAdaptor>::adj_edge_iterator adj_edge_iterator;
  typedef typename internal_adj_edge_iterator_selector<VGraph,
    EAdaptor>::const_adj_edge_iterator const_adj_edge_iterator;
  typedef va_iterator<typename VGraph::vertex_iterator, adj_edge_iterator,
      VAdaptor> vertex_iterator;
  typedef const_va_iterator<typename VGraph::vertex_iterator,
      const_adj_edge_iterator, VAdaptor> const_vertex_iterator;
  typedef typename VAdaptor::result_type vertex_property;
};

//////////////////////////////////////////////////////////////////////
/// @brief Selector to infer vertex iterator/property.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class VGraph, class EAdaptor>
struct internal_vertex_iterator_selector<VGraph, EAdaptor, use_default>
{
  typedef typename internal_adj_edge_iterator_selector<VGraph,
    EAdaptor>::adj_edge_iterator adj_edge_iterator;
  typedef typename internal_adj_edge_iterator_selector<VGraph,
    EAdaptor>::const_adj_edge_iterator const_adj_edge_iterator;
  typedef va_iterator<typename VGraph::vertex_iterator, adj_edge_iterator,
      use_default> vertex_iterator;
  typedef const_va_iterator<typename VGraph::vertex_iterator,
      const_adj_edge_iterator, use_default> const_vertex_iterator;
  typedef typename VGraph::vertex_property vertex_property;
};

//////////////////////////////////////////////////////////////////////
/// @brief Selector to infer vertex iterator/property.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class VGraph>
struct internal_vertex_iterator_selector<VGraph, use_default, use_default>
{
  typedef typename VGraph::vertex_iterator vertex_iterator;
  typedef typename VGraph::const_vertex_iterator const_vertex_iterator;
  typedef typename VGraph::vertex_property vertex_property;
};



//////////////////////////////////////////////////////////////////////
/// @brief A graph view using adaptors for vertex and edge properties.
/// @ingroup graph
/// @tparam VGraph The type of the input graph.
/// @tparam VAdaptor The type of the vertex property adaptor.
/// @tparam EAdaptor The type of the edge property adaptor.
//////////////////////////////////////////////////////////////////////
template <class VGraph, class VAdaptor = use_default,
    class EAdaptor = use_default>
class adjacency_graph_view
{
protected:
  VGraph& m_g;
  VAdaptor m_vadapt;
public:
  typedef typename VGraph::vertex_descriptor vertex_descriptor;

  //the following changes from the original graph depending on EAdaptor
  //new edge descriptor
  typedef typename internal_adj_edge_iterator_selector<VGraph,
    EAdaptor>::edge_descriptor edge_descriptor;
  //the new edge property
  typedef typename internal_adj_edge_iterator_selector<VGraph,
    EAdaptor>::edge_property edge_property;
  //new adj_edge iterators
  typedef typename internal_adj_edge_iterator_selector<VGraph,
    EAdaptor>::adj_edge_iterator adj_edge_iterator;
  typedef typename internal_adj_edge_iterator_selector<VGraph,
    EAdaptor>::const_adj_edge_iterator const_adj_edge_iterator;

  //the following changes from the original graph depending on VAdaptor
  typedef typename internal_vertex_iterator_selector<VGraph, EAdaptor,
    VAdaptor>::vertex_property vertex_property;
  //new vertex iterators
  typedef typename internal_vertex_iterator_selector<VGraph, EAdaptor,
    VAdaptor>::vertex_iterator vertex_iterator;
  typedef typename internal_vertex_iterator_selector<VGraph, EAdaptor,
    VAdaptor>::const_vertex_iterator const_vertex_iterator;

  typedef typename vertex_iterator::value_type vertex_reference;
  typedef typename const_vertex_iterator::value_type const_vertex_reference;

public:
  adjacency_graph_view(VGraph& g)
      : m_g(g)
  {
  }

  vertex_iterator begin()
  {
    return vertex_iterator(m_g.begin());
  }

  const_vertex_iterator begin() const
  {
    return const_vertex_iterator(static_cast<const VGraph&>(m_g).begin());
  }

  vertex_iterator end()
  {
    return vertex_iterator(m_g.end());
  }

  const_vertex_iterator end() const
  {
    return const_vertex_iterator(static_cast<const VGraph&>(m_g).end());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Selector to infer edge iterator/property.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class VGraph, class EAdaptor>
struct internal_edge_iterator_selector
{
  typedef ea_iterator<typename VGraph::edge_iterator, EAdaptor> edge_iterator;
  typedef const_ea_iterator<typename VGraph::edge_iterator,
    EAdaptor> const_edge_iterator;
};

//////////////////////////////////////////////////////////////////////
/// @brief Selector to infer edge iterator/property.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class VGraph>
struct internal_edge_iterator_selector<VGraph, use_default>
{
  typedef typename VGraph::edge_iterator edge_iterator;
  typedef typename VGraph::const_edge_iterator const_edge_iterator;
};



//////////////////////////////////////////////////////////////////////
/// @brief A static graph view using adaptors for vertex and edge properties.
/// @ingroup graph
/// @tparam VGraph The type of the input graph.
/// @tparam VAdaptor The type of the vertex property adaptor.
/// @tparam EAdaptor The type of the edge property adaptor.
//////////////////////////////////////////////////////////////////////
template <class VGraph, class VAdaptor = use_default,
    class EAdaptor = use_default>
class static_graph_view
  : public adjacency_graph_view<VGraph, VAdaptor, EAdaptor>
{
  typedef static_graph_view<VGraph, VAdaptor, EAdaptor> this_type;
  typedef adjacency_graph_view<VGraph, VAdaptor, EAdaptor> base_type;
public:
  typedef typename base_type::vertex_descriptor vertex_descriptor;
  typedef typename base_type::edge_descriptor edge_descriptor;

  typedef typename base_type::vertex_iterator vertex_iterator;
  typedef typename base_type::const_vertex_iterator const_vertex_iterator;
  typedef typename base_type::vertex_reference vertex_reference;
  typedef typename base_type::adj_edge_iterator adj_edge_iterator;
  typedef typename base_type::const_adj_edge_iterator const_adj_edge_iterator;
  typedef typename internal_edge_iterator_selector<VGraph,
    EAdaptor>::edge_iterator edge_iterator;
  typedef typename internal_edge_iterator_selector<VGraph,
    EAdaptor>::const_edge_iterator const_edge_iterator;

  static_graph_view(VGraph& g)
      : base_type(g)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator over all the edges of the graph.
  //////////////////////////////////////////////////////////////////////
  edge_iterator edges_begin()
  {
    return this->m_g.edges_begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator over all the edges of the graph.
  //////////////////////////////////////////////////////////////////////
  const_edge_iterator edges_begin() const
  {
    return this->m_g.edges_begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator over all the edges of the graph.
  //////////////////////////////////////////////////////////////////////
  edge_iterator edges_end()
  {
    return this->m_g.edges_end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator over all the edges of the graph.
  //////////////////////////////////////////////////////////////////////
  const_edge_iterator edges_end() const
  {
    return this->m_g.edges_end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the maximum descriptor for the vertex.
  //////////////////////////////////////////////////////////////////////
  size_t get_max_descriptor() const
  {
    return this->m_g.get_max_descriptor();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of vertices in this graph.
  //////////////////////////////////////////////////////////////////////
  size_t get_num_vertices() const
  {
    return this->m_g.get_num_vertices();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the number of edges in this graph.
  //////////////////////////////////////////////////////////////////////
  size_t get_num_edges() const
  {
    return this->m_g.get_num_edges();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the vertex with the specified descriptor, and returns a
  /// vertex_iterator pointing to it. If not found, the end of the graph is
  /// returned.
  /// @param vd Descriptor of the vertex.
  /// @return A vertex_iterator to the specified vertex, if found, or a
  /// vertex_iterator to the end of the graph otherwise.
  //////////////////////////////////////////////////////////////////////
  vertex_iterator find_vertex(vertex_descriptor const& vd)
  {
    return vertex_iterator(this->m_g.find_vertex(vd));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the vertex with the specified descriptor, and returns a
  /// vertex_iterator pointing to it. If not found, the end of the graph is
  /// returned.
  /// @param vd Descriptor of the vertex.
  /// @return A vertex_iterator to the specified vertex, if found, or a
  /// vertex_iterator to the end of the graph otherwise.
  //////////////////////////////////////////////////////////////////////
  const_vertex_iterator find_vertex(vertex_descriptor const& vd) const
  {
    typename VGraph::const_vertex_iterator vi =
        static_cast<const VGraph&>(this->m_g).find_vertex(vd);
    return const_vertex_iterator(vi);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the edge with the specified descriptor, and returns an
  /// iterator to its source vertex and an adj_edge_iterator pointing to the
  /// edge.
  /// @param ed Descriptor of the edge.
  /// @param vi vertex_iterator pointing to the source vertex of the edge,
  /// populated by the method.
  /// @param aei adj_edge_iterator pointing to the specified edge,
  /// populated by the method.
  /// @return Whether or not the edge was found.
  //////////////////////////////////////////////////////////////////////
  bool find_edge(edge_descriptor const& ed, vertex_iterator& vi,
                 adj_edge_iterator& ei)
  {
    typename VGraph::vertex_iterator o_vi;
    typename VGraph::adj_edge_iterator o_ei;
    typename VGraph::edge_descriptor n_ed(ed.source(), ed.target());
    bool res = this->m_g.find_edge(n_ed, o_vi, o_ei);
    vi = vertex_iterator(o_vi);
    ei = adj_edge_iterator(o_ei);
    return res;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds the edge with the specified descriptor, and returns an
  /// iterator to its source vertex and an adj_edge_iterator pointing to the
  /// edge.
  /// @param ed Descriptor of the edge.
  /// @param vi vertex_iterator pointing to the source vertex of the edge,
  /// populated by the method.
  /// @param aei adj_edge_iterator pointing to the specified edge,
  /// populated by the method.
  /// @return Whether or not the edge was found.
  //////////////////////////////////////////////////////////////////////
  bool find_edge(const edge_descriptor& ed, const_vertex_iterator& vi,
                 const_adj_edge_iterator& ei) const
  {
    typename VGraph::const_vertex_iterator o_vi;
    typename VGraph::const_adj_edge_iterator o_ei;
    typename VGraph::edge_descriptor n_ed(ed.source(), ed.target());
    bool res = this->m_g.find_edge(n_ed, o_vi, o_ei);
    vi = const_vertex_iterator(o_vi);
    ei = const_adj_edge_iterator(o_ei);
    return res;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper to infer the property of the base class based on the
/// current property and adaptor and add a vertex.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class OVGraph, class VAdaptor>
struct add_vertex_helper
{
  static bool add(OVGraph& g, typename VAdaptor::result_type& p)
  {
    typename VAdaptor::property_type op(p);
    return g.add_vertex(op);
  }

  static bool add(OVGraph& g, typename OVGraph::vertex_descriptor vd,
                  typename VAdaptor::result_type& p)
  {
    typename VAdaptor::property_type op(p);
    return g.add_vertex(vd, op);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper to infer the property of the base class based on the
/// current property and adaptor and add a vertex.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class OVGraph>
struct add_vertex_helper<OVGraph, use_default>
{
  static bool add(OVGraph& g, typename OVGraph::vertex_property& p)
  {
    return g.add_vertex(p);
  }

  static bool add(OVGraph& g, typename OVGraph::vertex_descriptor vd,
                  typename OVGraph::vertex_property& p)
  {
    return g.add_vertex(vd, p);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper to infer the property of the base class based on the
/// current edge descriptor and adaptor and add an edge.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class OVGraph, class NewED, class EAdaptor>
struct add_edge_helper
{
  static bool add(OVGraph& g, NewED& ed)
  {
    typename EAdaptor::edge_descriptor o_ed(ed);
    return g.add_edge(o_ed);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper to infer the property of the base class based on the
/// current edge descriptor and adaptor and add an edge.
/// @ingroup graphBaseUtil
//////////////////////////////////////////////////////////////////////
template <class OVGraph>
struct add_edge_helper<OVGraph, typename OVGraph::edge_descriptor, use_default>
{
  static bool add(OVGraph& g, typename OVGraph::edge_descriptor& ed)
  {
    return g.add_edge(ed);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A dynamic graph view using adaptors for vertex and edge properties.
/// @ingroup graph
/// @tparam VGraph The type of the input graph.
/// @tparam VAdaptor The type of the vertex property adaptor.
/// @tparam EAdaptor The type of the edge property adaptor.
//////////////////////////////////////////////////////////////////////
template <class VGraph, class VAdaptor = use_default,
    class EAdaptor = use_default>
class dynamic_graph_view
  : public static_graph_view<VGraph, VAdaptor, EAdaptor>
{
  typedef static_graph_view<VGraph, VAdaptor, EAdaptor> base_type;
  typedef dynamic_graph_view<VGraph, VAdaptor, EAdaptor> this_type;
public:
  typedef typename base_type::vertex_descriptor vertex_descriptor;
  typedef typename base_type::edge_descriptor edge_descriptor;

  typedef typename base_type::vertex_iterator vertex_iterator;
  typedef typename base_type::const_vertex_iterator const_vertex_iterator;
  typedef typename base_type::adj_edge_iterator adj_edge_iterator;
  typedef typename base_type::const_adj_edge_iterator const_adj_edge_iterator;
  typedef typename base_type::edge_iterator edge_iterator;
  typedef typename base_type::const_edge_iterator const_edge_iterator;

  typedef typename vertex_iterator::property_type vertex_property;
  typedef typename adj_edge_iterator::property_type edge_property;

  dynamic_graph_view(VGraph& g)
      : base_type(g)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add a vertex with the default property. The descriptor of this
  /// is generated automatically.
  /// @return The descriptor for the new vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(void)
  {
    return this->m_g.add_vertex();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add a vertex with the specified property. The descriptor of this
  /// is generated automatically.
  /// @return The descriptor for the new vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_property& vp)
  {
    //convert property to original view property
    return add_vertex_helper<VGraph, VAdaptor>::add(this->m_g, vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add a vertex with the specified descriptor and property.
  /// @return The descriptor for the new vertex.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor add_vertex(vertex_descriptor vd, vertex_property& vp)
  {
    return add_vertex_helper<VGraph, VAdaptor>::add(this->m_g, vd, vp);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an edge with given descriptor and default edge property.
  /// @param ed Descriptor of the desired edge.
  /// @return True if the edge was added, or false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool add_edge(edge_descriptor& ed)
  {
    return add_edge_helper<VGraph, edge_descriptor, EAdaptor>::add(this->m_g,
                                                                   ed);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clear this graph, deleting all its vertices and edges.
  //////////////////////////////////////////////////////////////////////
  void erase(void)
  {
    this->m_g.erase();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Delete the specified vertex.
  /// @param vd The vertex to be deleted.
  /// @return True if vertex was deleted, or false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool delete_vertex(vertex_descriptor const& vd)
  {
    return this->m_g.delete_vertex(vd);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Delete the specified edge.
  /// @param ed The edge to be deleted.
  /// @return True if edge was deleted, or false otherwise.
  //////////////////////////////////////////////////////////////////////
  bool delete_edge(edge_descriptor const& ed)
  {
    return this->m_g.delete_edge(ed);
  }
};

} // namespace stapl

#endif

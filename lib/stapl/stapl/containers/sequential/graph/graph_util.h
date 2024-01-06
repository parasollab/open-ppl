/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_UTIL_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_UTIL_HPP

#include <iostream>

#ifdef _STAPL
# include <stapl/runtime/serialization.hpp>
# include <tuple>
#endif

namespace stapl {

#define INVALID_VALUE ((size_t)-1)

//////////////////////////////////////////////////////////////////////
/// @brief Enumeration of graph attributes.
/// @ingroup pgraphImpl
///
/// Used to specify if the graph is DIRECTED or UNDIRECTED,
/// or if it allows multiple edges between the same source and target
/// vertices (MULTIEDGES) or not (NONMULTIEDGES).
//////////////////////////////////////////////////////////////////////
enum graph_attributes
{
  DIRECTED, UNDIRECTED, MULTIEDGES, NONMULTIEDGES
};


namespace properties {

//////////////////////////////////////////////////////////////////////
/// @brief Struct used to define graphs with no property for vertices
/// or edges.
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
struct no_property
{ };

} //namespace properties

//////////////////////////////////////////////////////////////////////
/// @brief Implementation of the edge descriptor.
/// @tparam VertexDescriptor The type of the vertex descriptor.
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
template <class VertexDescriptor>
class edge_descriptor_impl
{
public:
  typedef VertexDescriptor vertex_descriptor;
  typedef size_t           edge_id_type;

  edge_descriptor_impl(void)
    : m_source(),
      m_target(),
      m_id(INVALID_VALUE)
  { }

  edge_descriptor_impl(vertex_descriptor const& s,
                       vertex_descriptor const& t,
                       edge_id_type eid = INVALID_VALUE)
    : m_source(s),
      m_target(t),
      m_id(eid)
  { }

  vertex_descriptor source(void) const noexcept
  {
    return m_source;
  }

  vertex_descriptor target(void) const noexcept
  {
    return m_target;
  }

  edge_id_type id(void) const noexcept
  {
    return m_id;
  }

  void display(void) const
  {
    std::cout << "[" << m_source << "->" << m_target << ":" << m_id << "|";
  }

  bool operator==(edge_descriptor_impl const& other) const noexcept
  {
    if (this->m_source == other.m_source && this->m_target == other.m_target
        && this->m_id == other.m_id)
      return true;
    return false;
  }

  bool operator!=(edge_descriptor_impl const& other) const noexcept
  {
    if (this->m_source == other.m_source && this->m_target == other.m_target
        && this->m_id == other.m_id)
      return false;
    return true;
  }

#ifdef _STAPL
  typedef std::tuple<
            vertex_descriptor, vertex_descriptor, edge_id_type
          > member_types;

  void define_type(typer& t)
  {
    t.member(m_source);
    t.member(m_target);
    t.member(m_id);
  }
#endif

protected:
  /// The vertex descriptor of the source vertex for this edge.
  vertex_descriptor m_source;
  /// The vertex descriptor of the target vertex for this edge.
  vertex_descriptor m_target;
  /// @brief The unique id for this edge.
  ///
  /// This differentiates between multiple edges between the same source
  /// and target vertices.
  edge_id_type m_id;

  //////////////////////////////////////////////////////////////////////
  /// @brief Reverses the edge descriptor.
  /// @param t The edge descriptor.
  /// @return An edge descriptor with the same id as the input,
  /// but with source and target descriptors switched.
  //////////////////////////////////////////////////////////////////////
  friend edge_descriptor_impl reverse(edge_descriptor_impl const& t)
  {
    return edge_descriptor_impl(t.target(), t.source(), t.id());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Utility function to check the validity of an edge descriptor.
/// @ingroup pgraphImpl
///
/// This method is used to ensure there were no errors while executing
/// the methods returning edge descriptors (for example add_edge).
//////////////////////////////////////////////////////////////////////
template <class ED>
bool is_valid(ED const& ed)
{
  //typedef typename ED::vertex_descriptor VD;
  return ed.id() != (size_t) INVALID_VALUE ;
}

//////////////////////////////////////////////////////////////////////
/// @brief Utility function to check the validity of an edge descriptor.
/// @ingroup pgraphImpl
///
/// This method is used to ensure there were no errors while executing
/// the methods returning edge descriptors (for example add_edge).
//////////////////////////////////////////////////////////////////////
template <class ED>
bool is_source_valid(ED const& ed)
{
  typedef typename ED::vertex_descriptor VD;
  return ed.source() != VD(INVALID_VALUE );

}

//////////////////////////////////////////////////////////////////////
/// @brief Utility function to check the validity of an edge descriptor.
/// @ingroup pgraphImpl
///
/// This method is used to ensure there were no errors while executing
/// the methods returning edge descriptors (for example add_edge).
//////////////////////////////////////////////////////////////////////
template <class ED>
bool is_target_valid(ED const& ed)
{
  typedef typename ED::vertex_descriptor VD;
  return ed.target() != VD(INVALID_VALUE );

}

//////////////////////////////////////////////////////////////////////
/// @brief Utility function to check the validity of an edge descriptor.
/// @ingroup pgraphImpl
///
/// This method is used to ensure there were no errors while executing
/// the methods returning edge descriptors (for example add_edge).
//////////////////////////////////////////////////////////////////////
template <class ED>
bool is_id_valid(ED const& ed)
{
  //typedef typename ED::vertex_descriptor VD;
  return ed.id() != (size_t) INVALID_VALUE ;
}


//////////////////////////////////////////////////////////////////////
/// @brief A generator for vertex descriptors.
/// @ingroup pgraphImpl
///
/// This class will generate unique vertex descriptors, taking into
/// account that the graph may auto generate them, as well as the users
/// may specify them.
/// This is specialized for different types of vertex descriptors.
//////////////////////////////////////////////////////////////////////
template <class VD>
class vertex_descriptor_generator
{
  //generators for different vertex descriptor have to be provided
};


//////////////////////////////////////////////////////////////////////
/// @brief A generator for vertex descriptors, specialized for size_t.
/// @ingroup pgraphImpl
///
/// This class will generate unique vertex descriptors, taking into
/// account that the graph may auto generate them, as well as the users
/// may specify them.
//////////////////////////////////////////////////////////////////////
template <>
class vertex_descriptor_generator<size_t>
{
  size_t m_c;
public:
  typedef size_t vertex_descriptor;
  //invalid vertex descriptor
  static const size_t invalid_vd = INVALID_VALUE;


  //////////////////////////////////////////////////////////////////////
  /// @brief Create a generator which produces descriptors starting from
  /// the specified initial offset.
  //////////////////////////////////////////////////////////////////////
  vertex_descriptor_generator(size_t init = 0)
      : m_c(init)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Generate the next vertex descriptor
  //////////////////////////////////////////////////////////////////////
  size_t next()
  {
    return m_c++;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief integrate the vertex descriptor new_val into the current
  /// generated sequence.
  ///
  /// Used when the user has specified his/her own descriptor.
  //////////////////////////////////////////////////////////////////////
  void update(size_t new_val)
  {
    if (new_val > m_c)
      m_c = new_val;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Allows vertex descriptors to be reused. Not applicable for
  /// size_t vertex descriptors.
  //////////////////////////////////////////////////////////////////////
  void free(size_t /*vd*/)
  {
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reset this generator to start producing descriptors from zero (0).
  /// @todo Reset to the initial given value instead of zero (0).
  //////////////////////////////////////////////////////////////////////
  void reset()
  {
    m_c = 0;
  }

#ifdef _STAPL
  void define_type(typer& t)
  {
    t.member(m_c);
  }
#endif
};

//////////////////////////////////////////////////////////////////////
/// @brief An empty vertex descriptor generator. The vertex descriptors
/// will be controlled externally by the user.
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
template <class VD>
class external_descriptor_generator
{
public:
  typedef VD vertex_descriptor;
  static const size_t invalid_vd = INVALID_VALUE; //invalid vertex descriptor

  external_descriptor_generator(vertex_descriptor)
  {
  }

  size_t next()
  {
    return invalid_vd;
  }

  void update(size_t)
  {
  }

  void free(size_t)
  {
  }

  void reset()
  {
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief A simple search function for the graph.
/// @ingroup pgraphImpl
///
/// Searches the range from @p b to @p e, and returns the first place where
/// @p pred is satisfied.
/// Can't use std::find for certain operations, as the predicate will have
/// the iterator as argument and not the reference to the element
/// @param b An iterator to the beginning of the range.
/// @param e An iterator to the end of the range.
/// @param pred The predicate.
/// @return An iterator to the first element satisfying the predicate.
//////////////////////////////////////////////////////////////////////
template <class Iter, class Pred>
Iter graph_find(Iter b, Iter e, Pred const& pred)
{
  for (; b != e; ++b) {
    if (pred(b))
      return b;
  }
  return e;
}


//////////////////////////////////////////////////////////////////////
/// @brief Finds an edge with the specified user property.
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
template <class Property>
struct eq_property
{
  Property m_w;
  eq_property(Property const& w)
      : m_w(w)
  {
  }

  template <class Iter>
  bool operator()(Iter it) const
  {
    return (*it).property() == m_w;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Finds an edge with the specified target.
/// (for use with graph_find)
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
template <class VD>
struct eq_target
{
  VD m_vd;
  eq_target(VD const& v)
      : m_vd(v)
  {
  }

  template <class Iter>
  bool operator()(Iter it) const
  {
    return (*it).target() == m_vd;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Finds an edge with the specified target.
/// (for use with std::find_if)
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
template <class VD>
struct edge_eq_target
{
  VD m_vd;
  edge_eq_target(VD const& v)
    : m_vd(v)
  {
  }

  template <class Edge>
  bool operator()(Edge e) const
  {
    return e.target() == m_vd;
  }
};



//////////////////////////////////////////////////////////////////////
/// @brief Finds an edge with the specified edge id.
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
template <class EID>
struct eq_eid
{
  EID m_eid;
  eq_eid(EID const& v)
      : m_eid(v)
  {
  }

  template <class Iter>
  bool operator()(Iter it) const
  {
    return (*it).id() == m_eid;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Finds an edge with the specified edge id, source and destination.
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
template <class ED>
struct eq_ed
{
  ED m_ed;

  eq_ed(ED const& ed)
      : m_ed(ed)
  {
  }

  template <class Iter>
  bool operator()(Iter it) const
  {
    return ((*it).id() == m_ed.id()) && ((*it).source() == m_ed.source())
        && ((*it).target() == m_ed.target());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper to extract the property of a vertex/edge.
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
template <class Iterator, class Property>
struct helper_get_property
{
  static typename Iterator::property_type get(Iterator it)
  {
    return (*it).property();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Helper to extract the property of a vertex/edge. Specialized
/// for when the vertex/edge does not have a property.
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
template <class Iterator>
struct helper_get_property<Iterator, properties::no_property>
{
  static properties::no_property get(Iterator)
  {
    return properties::no_property();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Returns the property associated with an iterator (edge or vertex)
/// Returns no_property for when property not defined.
/// @ingroup pgraphImpl
//////////////////////////////////////////////////////////////////////
template <class Iterator>
typename Iterator::property_type get_property(Iterator it)
{
  return helper_get_property<Iterator,
                             typename Iterator::property_type>::get(it);
}


//////////////////////////////////////////////////////////////////////
/// @brief Copies a source graph into a destination graph,
/// handling cases for property which may or may not be there.
/// @ingroup pgraphImpl
/// @param source The source graph.
/// @param dest The destination graph.
//////////////////////////////////////////////////////////////////////
template <class Graph> //Graph models at least static_graph
void copy_graph(Graph& source, Graph& dest)
{
  for (typename Graph::vertex_iterator vi = source.begin();
       vi != source.end(); ++vi) {
    dest.add_vertex((*vi).descriptor(), stapl::get_property(vi));
  }
  typename Graph::edge_iterator ei = source.edges_begin();
  while (ei != source.edges_end()) {
    dest.add_edge(
        typename Graph::edge_descriptor((*ei).source(), (*ei).target()),
        stapl::get_property(ei));
    ++ei;
  }
}


//////////////////////////////////////////////////////////////////////
/// @brief Selector for extracting the vertex, given either a
/// vertex descriptor or a vertex reference.
/// @ingroup pgraphImpl
///
/// Used by the pGraph internal property map, where property is stored
/// internally on the vertex by the pGraph, providing efficient storage
/// and constant-time access to the vertex property.
//////////////////////////////////////////////////////////////////////
template <typename G, typename T>
struct i_vertex_selector
{
  typedef T result_type;
  static T extract(G*, T t)
  {
    return t;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Selector for extracting the vertex, given either a
/// vertex descriptor or a vertex reference, specialized for descriptor.
/// @ingroup pgraphImpl
///
/// Used by the pGraph internal property map, where property is stored
/// internally on the vertex by the pGraph, providing efficient storage
/// and constant-time access to the vertex property.
//////////////////////////////////////////////////////////////////////
template <typename G>
struct i_vertex_selector<G, typename G::vertex_descriptor>
{
  typedef typename G::vertex_descriptor vertex_descriptor;
  typedef typename G::vertex_reference vertex_reference;
  typedef vertex_reference result_type;

  static vertex_reference extract(G* g, vertex_descriptor vd)
  {
    return *(g->find_vertex(vd));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Selector for extracting the edge, given either an
/// edge descriptor or an edge reference.
/// @ingroup pgraphImpl
///
/// Used by the pGraph internal edge property map, where property is stored
/// internally on the edge by the pGraph, providing efficient storage
/// and constant-time access to the edge property.
//////////////////////////////////////////////////////////////////////
template <typename G, typename T>
struct i_edge_selector
{
  typedef T result_type;
  static T extract(G*, T t)
  {
    return t;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Selector for extracting the edge, given either an
/// edge descriptor or an edge reference, specialized for descriptor.
/// @ingroup pgraphImpl
///
/// Used by the pGraph internal edge property map, where property is stored
/// internally on the edge by the pGraph, providing efficient storage
/// and constant-time access to the edge property.
//////////////////////////////////////////////////////////////////////
template <typename G>
struct i_edge_selector<G, typename G::edge_descriptor>
{
  typedef typename G::edge_descriptor edge_descriptor;
  typedef typename G::edge_reference edge_reference;
  typedef typename G::vertex_reference vertex_reference;
  typedef typename G::adj_edge_iterator edge_iterator;

  typedef edge_reference result_type;

  static edge_reference extract(G* g, edge_descriptor ed)
  {
    return *(g->find_edge(ed));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Selector for extracting the vertex descriptor, given either a
/// vertex descriptor or a vertex reference.
/// @ingroup pgraphImpl
///
/// Used by the pGraph external property map, where property is stored
/// externally. Assumes descriptor based random-access in container.
//////////////////////////////////////////////////////////////////////
template <typename G, typename T>
struct e_selector
{
  static typename G::vertex_descriptor extract(T t)
  {
    return t.descriptor();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Selector for extracting the vertex descriptor, given either a
/// vertex descriptor or a vertex reference, specialized for descriptor.
/// @ingroup pgraphImpl
///
/// Used by the pGraph external property map, where property is stored
/// externally. Assumes descriptor based random-access in container.
//////////////////////////////////////////////////////////////////////
template <typename G>
struct e_selector<G, typename G::vertex_descriptor>
{
  typedef typename G::vertex_descriptor vertex_descriptor;
  static vertex_descriptor extract(vertex_descriptor vd)
  {
    return vd;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Selector for extracting the edge descriptor, given either an
/// edge descriptor or an edge reference, specialized for descriptor.
/// @ingroup pgraphImpl
///
/// Used by the pGraph external edge property map, where property is stored
/// externally. Assumes id based random-access in container.
//////////////////////////////////////////////////////////////////////
template <typename G>
struct e_selector<G, typename G::edge_descriptor>
{
  typedef typename G::edge_descriptor edge_descriptor;
  typedef typename edge_descriptor::edge_id_type edge_id_type;
  static edge_id_type extract(edge_descriptor ed)
  {
    return ed.id();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Selector for extracting the edge descriptor, given either an
/// edge descriptor or an edge reference, specialized for reference.
/// @ingroup pgraphImpl
///
/// Used by the pGraph external edge property map, where property is stored
/// externally. Assumes id based random-access in container.
//////////////////////////////////////////////////////////////////////
template <typename G>
struct e_selector<G, typename G::edge_reference>
{
  typedef typename G::edge_reference edge_reference;
  typedef typename edge_reference::edge_id_type edge_id_type;
  static edge_id_type extract(edge_reference er)
  {
    return er.id();
  }
};

} // namespace

#endif

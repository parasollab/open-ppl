/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_ALGO_UTIL_HPP
#define STAPL_CONTAINERS_SEQUENTIAL_GRAPH_GRAPH_ALGO_UTIL_HPP

#include <vector>
#include <map>
#include "../graph_util.h"

namespace stapl {

enum visitor_return {CONTINUE, EARLY_QUIT};

///////////////////////////////////////////////////////////////////////////
/// @brief A base class to be inherited by every visitor class.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoWf
///////////////////////////////////////////////////////////////////////////
template< class GRAPH>
class visitor_base
{
 typedef typename GRAPH::vertex_iterator   vertex_iterator;
 typedef typename GRAPH::adj_edge_iterator adj_edge_iterator;

 public:
  visitor_base() = default;

  ///////////////////////////////////////////////////////////////////////////
  /// @brief All the visitors will be related with a graph. This constructor
  /// can be used when you need to store the Graph associated with the
  /// current traversal/visitor.
  /// @todo Remove this method. It serves no purpose over the default
  /// constructor, but may be used.
  ///////////////////////////////////////////////////////////////////////////
  visitor_base(GRAPH& /*g*/)
  {
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called by the traversal when
  /// a vertex is reached for the first time.
  /// @param v Iterator to the vertex.
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return discover_vertex(vertex_iterator)
  {
    return CONTINUE;
  }


  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called when a vertex is reached and needs to be examined.
  /// @param v Iterator to the vertex.
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return examine_vertex(vertex_iterator)
  {
    return CONTINUE;
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called when an edge is reached and needs to be examined.
  /// @param v Iterator to the starting vertex of the edge.
  /// @param e Iterator to the edge (edge data and destination
  /// vertex).
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return examine_edge(vertex_iterator, adj_edge_iterator)
  {
    return CONTINUE;
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called when an edge whose
  /// destination is unvisited is reached.
  /// @param v Iterator to the starting vertex of the edge.
  /// @param e Iterator to the edge (edge data and destination
  /// vertex).
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return tree_edge(vertex_iterator, adj_edge_iterator)
  {
    return CONTINUE;
  }


  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called when an edge whose
  /// destination is already visited is reached.
  /// @param v Iterator to the starting vertex of the edge.
  /// @param e Iterator to the edge (edge data and destination
  /// vertex).
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return non_tree_edge(vertex_iterator,
                                              adj_edge_iterator)
  {
    return CONTINUE;
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called when an edge is reached whose
  /// destination is visited but not finished.
  /// @param v Iterator to the starting vertex of the edge.
  /// @param e Iterator to the edge (edge data and destination
  /// vertex).
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return gray_target(vertex_iterator, adj_edge_iterator)
  {
    return CONTINUE;
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called when an edge is reached whose
  /// destination is visited and finished.
  /// @param v Iterator to the starting vertex of the edge.
  /// @param e Iterator to the edge (edge data and destination
  /// vertex).
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return black_target(vertex_iterator,
                                             adj_edge_iterator)
  {
    return CONTINUE;
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called by traversal when a vertex becomes finished.
  /// @param v Iterator to the vertex.
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return finish_vertex(vertex_iterator, int =-1)
  {
    return CONTINUE;
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Destructor for the visitor_base class.
  ///////////////////////////////////////////////////////////////////////////
  virtual ~visitor_base() = default;
};


///////////////////////////////////////////////////////////////////////////
/// @brief A base class to be inherited by every visitor predecessors class.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoWf
///////////////////////////////////////////////////////////////////////////
template< class GRAPH>
class visitor_predecessors_base
{
  typedef typename GRAPH::vertex_iterator   vertex_iterator;
  typedef typename GRAPH::vertex_descriptor vertex_descriptor;

 public:
  visitor_predecessors_base() = default;

  ///////////////////////////////////////////////////////////////////////////
  /// @brief All the visitors will be related with a graph. This constructor
  /// can be used when you need to store the Graph associated with the
  /// current traversal/visitor.
  /// @todo Remove this method. It serves no purpose over the default
  /// constructor, but may be used.
  ///////////////////////////////////////////////////////////////////////////
  visitor_predecessors_base(GRAPH& g)
  {
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called by the traversal when
  /// a vertex is reached for the first time.
  /// @param v Iterator to the vertex.
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return discover_vertex(vertex_iterator)
  {
    return CONTINUE;
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called when a vertex is reached and needs to be examined.
  /// @param v Iterator to the vertex.
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return examine_vertex(vertex_iterator)
  {
    return CONTINUE;
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called when an edge is reached and needs to be examined.
  /// @param v Iterator to the starting vertex of the edge.
  /// @param e Descriptor of the target vertex of the edge.
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return examine_edge(vertex_iterator,
                                             vertex_descriptor)
  {
    return CONTINUE;
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called when an edge whose
  /// destination is unvisited is reached.
  /// @param v Iterator to the starting vertex of the edge.
  /// @param e Descriptor of the target vertex of the edge.
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return tree_edge(vertex_iterator, vertex_descriptor)
  {
    return CONTINUE;
  }


  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called when an edge whose
  /// destination is already visited is reached.
  /// @param v Iterator to the starting vertex of the edge.
  /// @param e Descriptor of the target vertex of the edge.
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return non_tree_edge(vertex_iterator,
                                              vertex_descriptor)
  {
    return CONTINUE;
  }


  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called when an edge is reached whose
  /// destination is visited but not finished.
  /// @param v Iterator to the starting vertex of the edge.
  /// @param e Descriptor of the target vertex of the edge.
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return gray_target(vertex_iterator, vertex_descriptor)
  {
    return CONTINUE;
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called when an edge is reached whose
  /// destination is visited and finished.
  /// @param v Iterator to the starting vertex of the edge.
  /// @param e Descriptor of the target vertex of the edge.
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return black_target(vertex_iterator,
                                             vertex_descriptor)
  {
    return CONTINUE;
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Will be called by traversal when a vertex becomes finished.
  /// @param v Iterator to the vertex.
  ///////////////////////////////////////////////////////////////////////////
  virtual visitor_return finish_vertex(vertex_iterator, int =-1)
  {
    return CONTINUE;
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Destructor for the visitor_predecessors_base class.
  ///////////////////////////////////////////////////////////////////////////
  virtual ~visitor_predecessors_base() = default;
};


///////////////////////////////////////////////////////////////////////////
/// @brief A color interface class for Breadth-First Search, Depth-First Search
/// and Dijkstra's algorithm.
/// @tparam CT The type chosen to represent the colors. A specialization must be
/// written by the user defining:
/// . static CT white() {...}
/// . static CT gray()  {...}
/// . static CT black() {...}
/// @ingroup seqGraphAlgoUtil
///////////////////////////////////////////////////////////////////////////
template <class CT>
struct graph_color
{
  typedef CT value_type;
  ///////////////////////////////////////////////////////////////////////////
  /// @brief The white color represents those targets that are not yet visited.
  ///////////////////////////////////////////////////////////////////////////
  static void white()
  {
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief The gray color represents those targets that have been visited,
  /// but are not yet finished.
  ///////////////////////////////////////////////////////////////////////////
  static void gray()
  {
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief The black color represents those targets that have been visited
  /// and are finished.
  ///////////////////////////////////////////////////////////////////////////
  static void black()
  {
  }
};


///////////////////////////////////////////////////////////////////////////
/// @brief A default specialization of the graph_color class with size_t
/// chosen as the value type to represent the colors.
/// @todo Move to a detail namespace to clean up stapl::sequential
/// @ingroup seqGraphAlgoUtil
///////////////////////////////////////////////////////////////////////////
template <>
struct graph_color<size_t>
{
  typedef size_t value_type;
  static value_type white()
  {
    return 0;
  }
  static value_type gray()
  {
    return 1;
  }
  static value_type black()
  {
    return 2;
  }
};



//////////////////////GRAPH PROPERTY_MAP////////////////////////////////

//identity functor for the property maps.

///////////////////////////////////////////////////////////////////////////
/// @brief The identity functor for graph property maps.
/// @tparam Property The type of the graph property.
/// @todo We have a typing problem when the identity function is templated on
/// bool and is used with an external property map where the container is
/// std::vector<bool>, because of the STL's specialization. The STL
/// specialization returns an object of type _Bit_reference, which doesn't play
/// nicely with the ident_prop_func as-is.
/// @ingroup seqGraphAlgoUtil
///////////////////////////////////////////////////////////////////////////
template <class Property>
class ident_prop_func
{
public:
  typedef Property value_type;

  value_type get(Property& p)
  {
    return p;
  }

  void       put(Property& p, value_type v)
  {
    p = v;
  }

  template <class Functor>
  void apply(Property& p, Functor f)
  {
    f(p);
  }
};


namespace sequential {

///////////////////////////////////////////////////////////////////////////
/// @brief The vertex property map class, which maps a set of vertices to
/// a set of properties.
/// @tparam G The graph class for the property map.
/// @tparam Functor The functor for the property map, which extracts a
/// sub-property from a property class.
/// @ingroup seqGraphAlgoUtil
///////////////////////////////////////////////////////////////////////////
template <class G, class Functor=ident_prop_func<typename G::vertex_property> >
class vertex_property_map
{
  G*      m_g;
  Functor m_f;

 public:
  typedef typename Functor::value_type    property_value_type;

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Constructor for the vertex_property_map class. Takes in a graph
  /// and a functor, and stores them as data members.
  ///////////////////////////////////////////////////////////////////////////
  vertex_property_map(G& g, Functor f)
    : m_g(&g), m_f(f)
  {
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Constructor for the vertex_property_map class. No functor is
  /// specified, so a default functor of type Functor is used.
  ///////////////////////////////////////////////////////////////////////////
  vertex_property_map(G& g)
    : m_g(&g), m_f(Functor())
  {
  }

  ////////////////////////////////////////////////////////////////////////////
  /// @brief Applies an alternate functor to a vertex in the map.
  /// @param v The vertex to apply the functor to.
  /// @param f2 The alternate functor.
  ///////////////////////////////////////////////////////////////////////////
  template<typename Vertex, typename Functor2>
  void apply(Vertex v, Functor2 f2)
  {
    m_f.apply(i_vertex_selector<G, Vertex>::
             extract(m_g, v).property(), f2);
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Changes the property of a vertex to the provided property.
  /// @param v The desired vertex.
  /// @param c The new property to assign to v.
  ///////////////////////////////////////////////////////////////////////////
  template<typename Vertex>
  void put(Vertex v, property_value_type c)
  {
    m_f.put(i_vertex_selector<G, Vertex>::
           extract(m_g, v).property(), c);
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Returns the property of a given vertex.
  /// @param v The vertex whose property will be returned.
  ///////////////////////////////////////////////////////////////////////////
  template<typename Vertex>
  property_value_type get(Vertex v)
  {
    return m_f.get(i_vertex_selector<G, Vertex>::
                  extract(m_g, v).property());
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Resets the properties of all vertices in the graph to a default
  /// property.
  ///////////////////////////////////////////////////////////////////////////
  void reset()
  {
    typename G::vertex_iterator it, it_end;
    it = m_g->begin();
    it_end = m_g->end();
    for (; it != it_end; ++it)
      m_f.put((*it).property(), property_value_type());
  }
};

///////////////////////////////////////////////////////////////////////////
/// @brief The edge property map that maps each edge in a set to an edge
/// property.
/// @tparam G The graph for the property map.
/// @tparam Functor The functor class for the property map.
/// @ingroup seqGraphAlgoUtil
///////////////////////////////////////////////////////////////////////////
template <class G, class Functor=ident_prop_func<typename G::edge_property> >
class edge_property_map
{
  G*      m_g;
  Functor m_f;

 public:
  typedef typename Functor::value_type property_value_type;

  // view constructors.
  ///////////////////////////////////////////////////////////////////////////
  /// @brief A constructor for the edge property map, where the graph and
  /// the functor are specified and stored as data members.
  ///////////////////////////////////////////////////////////////////////////
  edge_property_map(G& g, Functor f)
    : m_g(&g), m_f(f)
  {
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief A constructor for the edge property map where the functor is not
  /// specified. A default functor of type Functor is used.
  ///////////////////////////////////////////////////////////////////////////s
  edge_property_map(G& g)
    : m_g(&g), m_f(Functor())
  {
  }

  ////////////////////////////////////////////////////////////////////////////
  /// @brief Applies an alternate functor to an edge in the map.
  /// @param e The edge to apply the alternate functor to.
  /// @param f2 The alternate functor.
  ///////////////////////////////////////////////////////////////////////////
  template<typename Edge, typename Functor2>
  void apply(Edge e, Functor2 f2)
  {
    m_f.apply(i_edge_selector<G, Edge>::
             extract(m_g, e).property(), f2);
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Changes the property of an edge to the provided property.
  /// @param e The desired edge.
  /// @param c The new property to assign to e.
  ///////////////////////////////////////////////////////////////////////////
  template<typename Edge>
  void put(Edge e, property_value_type c)
  {
    m_f.put(i_edge_selector<G, Edge>::
           extract(m_g, e).property(), c);
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Returns the property of a given edge.
  /// @param e The edge whose property will be returned.
  ///////////////////////////////////////////////////////////////////////////
  template<typename Edge>
  property_value_type get(Edge e)
  {
    return m_f.get(i_edge_selector<G, Edge>::
                  extract(m_g, e).property());
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Resets the properties of each edge to a default property.
  ///////////////////////////////////////////////////////////////////////////
  void reset()
  {
    typename G::edge_iterator it, it_end;
    it = m_g->edges_begin();
    it_end = m_g->edges_end();
    for (; it != it_end; ++it)
      m_f.put((*it).property(), property_value_type());
  }
};

///////////////////////////////////////////////////////////////////////////
/// @brief The graph external property map class, which maps each element
/// in a container to a property.
/// @tparam G The graph type for the map.
/// @tparam C The container type for the property map.
/// @tparam Functor The functor class for the property map.
/// @ingroup seqGraphAlgoUtil
///////////////////////////////////////////////////////////////////////////
template <class G, class C, class Functor=ident_prop_func<
                                           typename C::value_type> >
class graph_external_property_map
{
protected:
  C*      m_c;
  Functor m_f;

public:
  typedef typename Functor::value_type property_value_type;

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Constructor, which takes the container and functor as parameters
  /// and sets them as data members.
  ///////////////////////////////////////////////////////////////////////////
  graph_external_property_map(C& c, Functor f)
    : m_c(&c), m_f(f)
  {
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Constructor which takes the container and sets it as a data member.
  /// No functor is given, so a default one of type Functor is used.
  ///////////////////////////////////////////////////////////////////////////
  graph_external_property_map(C& c)
    : m_c(&c), m_f(Functor())
  {
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Applies an alternate functor to the specified object.
  /// @param o The object to apply the alternate functor to.
  /// @param f2 The alternate functor.
  ///////////////////////////////////////////////////////////////////////////
  template<typename Object, typename Functor2>
  void apply(Object o, Functor2 f2)
  {
    m_f.apply((*m_c)[e_selector<G, Object>::extract(o)], f2);
  }


  ///////////////////////////////////////////////////////////////////////////
  /// @brief Assigns a property to the specified object.
  /// @param o The object to assign a property to.
  /// @param v The property to assign.
  ///////////////////////////////////////////////////////////////////////////
  template<typename Object>
  void put(Object o, property_value_type c)
  {
    m_f.put((*m_c)[e_selector<G, Object>::extract(o)], c);
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Returns the property of a given object.
  /// @param o The object whose property will be returned.
  ///////////////////////////////////////////////////////////////////////////
  template<typename Object>
  property_value_type get(Object o)
  {
    return m_f.get((*m_c)[e_selector<G, Object>::extract(o)]);
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Resets the properties of each object in the container to a default
  /// property.
  ///////////////////////////////////////////////////////////////////////////
  void reset()
  {
    typename C::iterator it, it_end;
    it = m_c->begin();
    it_end = m_c->end();
    for ( ; it != it_end; ++it)
      (*it) = property_value_type();
  }
};


///////////////////////////////////////////////////////////////////////////
/// @brief A specialized external property map class, which uses a vector as
/// the container type.
/// @tparam G The graph type for the property map.
/// @tparam T The type of object held in the vector.
/// @tparam Functor The functor class for the property map.
/// @ingroup seqGraphAlgoUtil
///////////////////////////////////////////////////////////////////////////
template <class G, class T, class Functor=ident_prop_func<T> >
class vector_property_map
  : public graph_external_property_map<G,std::vector<T>,Functor>
{
  typedef std::vector<T>                                        container_type;
  typedef graph_external_property_map<G,container_type,Functor> base_type;

 public:
  typedef typename base_type::property_value_type property_value_type;

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Basic constructor, calls graph_external_property constructor
  /// with a container of type container_type.
  ///////////////////////////////////////////////////////////////////////////
  vector_property_map()
    : base_type(m_vector)
  {
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Constructor which specifies the functor to be used. Calls the
  /// constructor of graph_external_property with a container of type
  /// container_type and the functor provided.
  ///////////////////////////////////////////////////////////////////////////
  vector_property_map(Functor f)
    : base_type(m_vector, f)
  {
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Assigns a property to the specified object, resizing the vector
  /// if needed.
  /// @param o The object to assign a property to.
  /// @param v The property to assign.
  ///////////////////////////////////////////////////////////////////////////
  template<typename Object>
  void put(Object o, property_value_type v)
  {
    typename G::vertex_descriptor vd = e_selector<G,Object>::extract(o);
    if (vd >= m_vector.size())
      m_vector.resize(vd+1);
    this->m_f.put(m_vector[vd], v);
  }

  ///////////////////////////////////////////////////////////////////////////
  /// @brief Returns the property of a given object, or a default property if
  /// the object doesn't have an associated property.
  /// @param o The object whose property will be returned.
  ///////////////////////////////////////////////////////////////////////////
  template<typename Object>
  property_value_type get(Object o)
  {
    typename G::vertex_descriptor vd = e_selector<G,Object>::extract(o);
    if (vd < m_vector.size())
      return this->m_f.get(m_vector[vd]);
    else
      return property_value_type();
  }


  ///////////////////////////////////////////////////////////////////////////
  /// @brief Applies an alternate functor to the specified object, resizing
  /// the vector if needed.
  /// @param o The object to apply the alternate functor to.
  /// @param f2 The alternate functor.
  ///////////////////////////////////////////////////////////////////////////
  template<typename Object, typename Functor2>
  void apply(Object o, Functor2 f2)
  {
    typename G::vertex_descriptor vd = e_selector<G,Object>::extract(o);
    if (vd >= m_vector.size())
      m_vector.resize(vd+1);
    this->m_f.apply(m_vector[vd], f2);
  }

 private:
  container_type m_vector;
};

///////////////////////////////////////////////////////////////////////////
/// @brief A specialized external property map class, which uses a std::mad
/// as the container type.
/// @tparam G The graph type for the property map.
/// @tparam T The type of object held by the map.
/// @tparam Functor The functor for the property map.
/// @ingroup seqGraphAlgoUtil
///////////////////////////////////////////////////////////////////////////
template <class G, class T, class Functor=ident_prop_func<T> >
class map_property_map
  : public graph_external_property_map<G,
             std::map<typename G::vertex_descriptor, T>, Functor>
{
  typedef std::map<typename G::vertex_descriptor, T>            container_type;
  typedef graph_external_property_map<G,container_type,Functor> base_type;

public:
  typedef typename base_type::property_value_type property_value_type;

  ///////////////////////////////////////////////////////////////////////////
  /// @brief A constructor, which calls the graph_external_property_map
  /// constructor with a container of type container_type.
  ///////////////////////////////////////////////////////////////////////////
  map_property_map()
    : base_type(m_map)
  {
  }

  //////////////////////////////////////////////////////////////////////////
  /// @brief A constructor, which calls the graph_external_property_map
  /// constructor with a container of type container_type and the specified
  /// functor.
  ///////////////////////////////////////////////////////////////////////////
  map_property_map(Functor f)
    : base_type(m_map, f)
  {
  }

  //put, get, and apply from the base class are sufficient.


  //////////////////////////////////////////////////////////////////////////
  /// @brief Resets the properties of each vertex to a default property.
  ///////////////////////////////////////////////////////////////////////////
  void reset()
  {
    typename container_type::iterator it, it_end;
    it = m_map.begin();
    it_end = m_map.end();
    for ( ; it != it_end; ++it)
      (*it).second = property_value_type();
  }

private:
  container_type m_map;
};

}//namespace sequential

}//namespace stapl

#endif

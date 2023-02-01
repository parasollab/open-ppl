/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_PROPERTY_MAPS_HPP
#define STAPL_CONTAINERS_GRAPH_PROPERTY_MAPS_HPP

#include <stapl/containers/sequential/graph/graph_util.h>

namespace stapl {

// uses struct i_vertex_selector defined in graph/graph_util.h

//////////////////////////////////////////////////////////////////////
/// @brief Internal property map to access vertex properties stored on
/// graph vertices.
/// @tparam PG The graph view.
/// @tparam Functor Functor to extract the desired property from the vertex
/// property.
///
/// Functor is used to extract a sub-property from a property class.
/// It should have following methods:
///  . typedef value_type;
///  . value_type get(Property p);
///  . void put(Property p, value_type c);
///  . apply(Property p, functor f);
//////////////////////////////////////////////////////////////////////
template <class PG, class Functor>
class graph_internal_property_map
{
  typedef typename PG::vertex_descriptor           vertex_descriptor;
  typedef typename PG::vertex_reference            vertex_reference;

  /// The graph view.
  PG* m_graph;

  /// Functor used to extract the desired property from an edge property.
  Functor m_f;

 public:
  typedef typename PG::reference                   reference;
  typedef PG                                       view_type;
  typedef typename Functor::value_type             value_type;
  typedef typename Functor::value_type             property_value_type;

  graph_internal_property_map(view_type& graph)
    : m_graph(&graph), m_f()
  { }

  graph_internal_property_map(view_type& graph, Functor f)
    : m_graph(&graph), m_f(f)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the specified functor to the property associated with
  /// the provided vertex.
  /// @param v The vertex associated with the property. Can either be a
  /// vertex object/proxy/reference or a vertex descriptor.
  /// @param f2 The functor to apply.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex, typename Functor2>
  void apply(Vertex v, Functor2 f2)
  {
    typename i_vertex_selector<PG, Vertex>::result_type pref =
      i_vertex_selector<PG, Vertex>::extract(m_graph, v);
    m_f.apply(pref.property(), f2);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the property associated with the provided vertex to the
  /// provided value.
  /// @param v The vertex associated with the property. Can either be a
  /// vertex object/proxy/reference or a vertex descriptor.
  /// @param c The value of the property.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex>
  void put(Vertex v, value_type c)
  {
    typedef typename
      i_vertex_selector<PG, Vertex>::result_type::property_reference pref_t;
    typename i_vertex_selector<PG, Vertex>::result_type pref =
      i_vertex_selector<PG, Vertex>::extract(m_graph, v);
    m_f.template put<pref_t>(pref.property(), c);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the property associated with the provided vertex.
  /// @param v The vertex associated with the property. Can either be a
  /// vertex object/proxy/reference or a vertex descriptor.
  /// @return The value of the property.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex>
  value_type get(Vertex v)
  {
    typename i_vertex_selector<PG, Vertex>::result_type pref =
      i_vertex_selector<PG, Vertex>::extract(m_graph, v);
    return m_f.get(pref.property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Not used. Provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  void reset()
  { }

  void define_type(typer& t)
  {
    t.member(m_graph);
    t.member(m_f);
  }

  view_type* get_view()
  { return m_graph; }

  view_type& view() const
  { return *m_graph; }

  bool is_local() const
  { return true; }

  void pre_execute()
  { }

  void post_execute()
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief External property map to store vertex properties.
/// @tparam PG The graph view.
/// @tparam Property Property type of the vertex.
/// @tparam View Type of the view for storing external properties.
//////////////////////////////////////////////////////////////////////
template <class PG, class Property, class View>
class graph_external_property_map
{
public:
  typedef typename PG::vertex_descriptor           vertex_descriptor;
  typedef typename PG::vertex_reference            vertex_reference;
  typedef typename PG::reference                   reference;
  typedef Property            value_type;
  typedef View                view_type;

private:
  /// The view storing the properties.
  view_type m_props;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an external vertex property map for the given graph
  /// using the provided view as storage.
  /// @param PG The graph view.
  /// @param props The external view where properties will be stored.
  //////////////////////////////////////////////////////////////////////
  graph_external_property_map(PG const&, view_type const& props)
    : m_props(props)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the specified functor to the property associated with
  /// the provided vertex.
  /// @param v The vertex associated with the property. Can either be a
  /// vertex object/proxy/reference or a vertex descriptor.
  /// @param f2 The functor to apply.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex, typename Functor2>
  void apply(Vertex v, Functor2 f2)
  {
    m_props.apply_set(e_selector<PG, Vertex>::extract(v), f2);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the property associated with the provided vertex to the
  /// provided value.
  /// @param v The vertex associated with the property. Can either be a
  /// vertex object/proxy/reference or a vertex descriptor.
  /// @param c The value of the property.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex>
  void put(Vertex v, value_type c)
  {
    m_props[e_selector<PG, Vertex>::extract(v)] = c;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the property associated with the provided vertex.
  /// @param v The vertex associated with the property. Can either be a
  /// vertex object/proxy/reference or a vertex descriptor.
  /// @return The value of the property.
  //////////////////////////////////////////////////////////////////////
  template<typename Vertex>
  value_type get(Vertex v) const
  {
    return m_props[e_selector<PG, Vertex>::extract(v)];
  }

  bool is_local() const
  { return m_props.is_local(); }

  void pre_execute()
  { m_props.pre_execute(); }

  void post_execute()
  { m_props.post_execute(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Not used. Provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  void reset()
  {
    //::stapl::map_func()
  }

  void define_type(typer& t)
  {
    t.member(m_props);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Internal property map to access edge properties stored on
/// graph edges.
/// @tparam PG The graph view.
/// @tparam Functor Functor to extract the desired property from the edge
/// property.
///
/// Functor is used to extract a sub-property from a property class.
/// It should have following methods:
///  . typedef value_type;
///  . value_type get(Property p);
///  . void put(Property p, value_type c);
///  . apply(Property p, functor f);
//////////////////////////////////////////////////////////////////////
template <class PG, class Functor>
class graph_edge_property_map
{
  typedef typename PG::vertex_descriptor  vertex_descriptor;
  typedef typename PG::vertex_reference   vertex_reference;

  /// The graph view.
  PG* m_graph;

  /// Functor used to extract the desired property from an edge property.
  Functor m_f;

public:
  typedef typename PG::reference                   reference;
  typedef PG                                       view_type;
  typedef typename Functor::value_type             value_type;


  graph_edge_property_map(view_type& graph)
    : m_graph(&graph), m_f(Functor())
  { }

  graph_edge_property_map(view_type& graph, Functor f)
    : m_graph(&graph), m_f(f)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the specified functor to the property associated with
  /// the provided edge.
  /// @param e The edge associated with the property. Can either be an
  /// edge object/proxy/reference or an edge descriptor.
  /// @param f2 The functor to apply.
  /// @fixme This should work for edge descriptors also.
  //////////////////////////////////////////////////////////////////////
  template <typename Edge, typename Functor2>
  void apply(Edge& e, Functor2 f2)
  {
    // typename i_edge_selector<PG, Edge>::result_type pref =
    //   i_edge_selector<PG, Edge>::extract(m_graph, e);
    // m_f.apply(pref.property(), f2);
    m_f.apply(e.property(), f2);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the property associated with the provided edge to the
  /// provided value.
  /// @param e The edge associated with the property. Can either be an
  /// edge object/proxy/reference or an edge descriptor.
  /// @param c The value of the property.
  /// @fixme This should work for edge descriptors also.
  //////////////////////////////////////////////////////////////////////
  template <typename Edge>
  void put(Edge& e, value_type c)
  {
    typedef typename i_edge_selector<PG, Edge>::result_type::property_reference
      pref_t;
    // typename i_edge_selector<PG, Edge>::result_type pref =
    //   i_edge_selector<PG, Edge>::extract(m_graph, e);
    // m_f.template put<pref_t> (pref.property(), c);
    m_f.template put<pref_t>(e.property(), c);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the property associated with the provided edge.
  /// @param e The edge associated with the property. Can either be an
  /// edge object/proxy/reference or an edge descriptor.
  /// @return The value of the property.
  /// @fixme This should work for edge descriptors also.
  //////////////////////////////////////////////////////////////////////
  template <typename Edge>
  value_type get(Edge const& e)
  {
    // typename i_edge_selector<PG, Edge>::result_type pref =
    //   i_edge_selector<PG, Edge>::extract(m_graph, e);
    // return m_f.get(pref.property());
    return m_f.get(e.property());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Not used. Provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  void reset()
  { }

  void define_type(typer& t)
  {
    t.member(m_graph);
    t.member(m_f);
  }

};


//////////////////////////////////////////////////////////////////////
/// @brief External property map to store edge properties.
/// @tparam PG The graph view.
/// @tparam Property Property type of the edge.
/// @tparam View Type of the view for storing external properties.
//////////////////////////////////////////////////////////////////////
template <class PG, class Property, class View>
class graph_external_edge_property_map
{
public:
  typedef typename PG::vertex_descriptor           vertex_descriptor;
  typedef typename PG::vertex_reference            vertex_reference;
  typedef typename PG::reference                   reference;
  typedef Property            value_type;
  typedef View                view_type;

private:
  /// The view to store properties.
  view_type m_props;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates an external edge property map for the given graph
  /// using the provided view as storage.
  /// @param PG The graph view.
  /// @param props The external view where properties will be stored.
  //////////////////////////////////////////////////////////////////////
  graph_external_edge_property_map(PG const& graph, view_type const& props)
    : m_props(props)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Applies the specified functor to the property associated with
  /// the provided edge.
  /// @param e The edge associated with the property. Can either be an
  /// edge object/proxy/reference or an edge descriptor.
  /// @param f2 The functor to apply.
  //////////////////////////////////////////////////////////////////////
  template<typename Edge, typename Functor2>
  void apply(Edge e, Functor2 f2)
  {
    m_props.apply_set(e_selector<PG, Edge>::extract(e), f2);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Sets the property associated with the provided edge to the
  /// provided value.
  /// @param e The edge associated with the property. Can either be an
  /// edge object/proxy/reference or an edge descriptor.
  /// @param c The value of the property.
  //////////////////////////////////////////////////////////////////////
  template<typename Edge>
  void put(Edge e, value_type c)
  {
    m_props[e_selector<PG,
            typename PG::edge_descriptor>::extract(e.descriptor())]
      = c;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the property associated with the provided edge.
  /// @param e The edge associated with the property. Can either be an
  /// edge object/proxy/reference or an edge descriptor.
  /// @return The value of the property.
  //////////////////////////////////////////////////////////////////////
  template<typename Edge>
  value_type get(Edge e)
  {
    return m_props[
      e_selector<PG, typename PG::edge_descriptor>::extract(e.descriptor())];
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Not used. Provided for compatibility.
  //////////////////////////////////////////////////////////////////////
  void reset()
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref graph_internal_property_map
/// @tparam PG The graph view.
/// @tparam Functor Functor to extract the desired property from the vertex
/// property.
/// @tparam Accessor Type of the accessor for the proxy.
//////////////////////////////////////////////////////////////////////
template <class PG, class Functor, typename Accessor>
class proxy<graph_internal_property_map<PG, Functor>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef graph_internal_property_map<PG, Functor> target_t;

 public:
  typedef typename target_t::reference                   reference;
  typedef PG                                             view_type;
  typedef typename Functor::value_type                   value_type;


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

  // property-map methods.
  template<typename Vertex, typename Functor2>
  void apply(Vertex v, Functor2 f2)
  { Accessor::read().apply(v, f2); }

  template<typename Vertex>
  void put(Vertex v, value_type c)
  { Accessor::read().put(v, c); }

  template<typename Vertex>
  value_type get(Vertex v)
  { return Accessor::read().get(v); }

  void reset() { }

  view_type* get_view()
  { return Accessor::read().get_view(); }

  view_type& view() const
  { return Accessor::read().view(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref graph_external_property_map
/// @tparam PG The graph view.
/// @tparam Property Property type of the vertex.
/// @tparam View Type of the View for storing external properties.
/// @tparam Accessor Type of the accessor for the proxy.
//////////////////////////////////////////////////////////////////////
template <class PG, class Property, class View, typename Accessor>
class proxy<graph_external_property_map<PG, Property, View>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef graph_external_property_map<PG, Property, View>      target_t;

 public:
  typedef typename target_t::reference                         reference;
  typedef View                                                 view_type;
  typedef Property                                             value_type;


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

  // property-map methods.
  template<typename Vertex, typename Functor2>
  void apply(Vertex v, Functor2 f2)
  { Accessor::read().apply(v, f2); }

  template<typename Vertex>
  void put(Vertex v, value_type c)
  { Accessor::read().put(v, c); }

  template<typename Vertex>
  value_type get(Vertex v)
  { return Accessor::read().get(v); }

  void reset()
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for @ref graph_edge_property_map
/// @tparam PG The graph view.
/// @tparam Functor Functor to extract the desired property from the edge
/// property.
/// @tparam Accessor Type of the accessor for the proxy.
//////////////////////////////////////////////////////////////////////
template <class PG, class Functor, typename Accessor>
class proxy<graph_edge_property_map<PG, Functor>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef graph_edge_property_map<PG, Functor> target_t;

 public:
  typedef typename target_t::reference                   reference;
  typedef PG                                             view_type;
  typedef typename Functor::value_type                   value_type;


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

  // property-map methods.
  template<typename Edge, typename Functor2>
  void apply(Edge& e, Functor2 f2)
  { Accessor::read().apply(e, f2); }

  template<typename Edge>
  void put(Edge& e, value_type c)
  { Accessor::read().put(e, c); }

  template<typename Edge>
  value_type get(Edge& e)
  { return Accessor::read().get(e); }

  void reset()
  { }

  view_type* get_view()
  { return Accessor::read().get_view(); }

  view_type& view() const
  { return Accessor::read().view(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref proxy for
/// @ref graph_external_edge_property_map
/// @tparam PG The graph view.
/// @tparam Property Property type of the edge.
/// @tparam View Type of the View for storing external properties.
/// @tparam Accessor Type of the accessor for the proxy.
//////////////////////////////////////////////////////////////////////
template <class PG, class Property, class View, typename Accessor>
class proxy<graph_external_edge_property_map<PG, Property, View>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef graph_external_edge_property_map<PG, Property, View> target_t;

 public:
  typedef typename target_t::reference                         reference;
  typedef View                                                 view_type;
  typedef Property                                             value_type;


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

  // property-map methods.
  template<typename Edge, typename Functor2>
  void apply(Edge e, Functor2 f2)
  { Accessor::read().apply(e, f2); }

  template<typename Edge>
  void put(Edge e, value_type c)
  { Accessor::read().put(e, c); }

  template<typename Edge>
  value_type get(Edge e)
  { return Accessor::read().get(e); }

  void reset()
  { }
};

} // stapl namespace

#endif /* STAPL_CONTAINERS_GRAPH_PROPERTY_MAPS_HPP */

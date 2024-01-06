/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATOR_BASE_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATOR_BASE_HPP

#include <stapl/containers/graph/dynamic_graph.hpp>
#include <boost/type_traits/is_base_of.hpp>

#include <stapl/utility/random.hpp>
#include <boost/random/uniform_int_distribution.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which adds vertices to a generated graph with the default
///   property.
///
/// This is used when the user does not provide a vertex generation functor.
//////////////////////////////////////////////////////////////////////
struct populate_vertices
{
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param x Id of the vertex to add.
  /// @param view View over the graph to which the vertex is added.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename V>
  void operator()(T x, V& view)
  {
    view.add_vertex(x, typename V::vertex_property());
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor used to call the provided vertex addition functor.
/// @tparam WF The vertex addition functor to call.
/// @tparam bool Flag to select correct specialization based on if the target
///   graph is dynamic (true) or not.
//////////////////////////////////////////////////////////////////////
template<typename WF, bool>
struct av_helper
{ };

//////////////////////////////////////////////////////////////////////
/// @brief @copybrief av_helper
/// @tparam WF The vertex addition functor to call.
///
/// This is the specialization called for stapl::dynamic_graph. It adds N
/// vertices to the given graph in parallel using the provided functor.
//////////////////////////////////////////////////////////////////////
template<typename WF>
struct av_helper<WF, true>
{
  WF m_wf;
  size_t m_num_vertices;

  //////////////////////////////////////////////////////////////////////
  /// @param wf The vertex addition functor.
  /// @param num_vertices The number of vertices to add.
  //////////////////////////////////////////////////////////////////////
  av_helper(WF const& wf, size_t num_vertices)
    : m_wf(wf), m_num_vertices(num_vertices)
  { }

  template<typename G>
  void operator()(G& g)
  {
    map_func(m_wf, counting_view<size_t>(m_num_vertices), make_repeat_view(g));
  }

  void define_type(typer& t)
  {
    t.member(m_wf);
    t.member(m_num_vertices);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief @copybrief av_helper
/// @tparam WF The vertex addition functor to call.
///
/// This is the specialization called when the input is not a
/// stapl::dynamic_graph. It performs a no-op.
//////////////////////////////////////////////////////////////////////
template<typename WF>
struct av_helper<WF, false>
{
  av_helper(WF const&, size_t)
  { }

  template<typename G>
  void operator()(G& g)
  { }
};


//////////////////////////////////////////////////////////////////////
/// @brief Type-traits metafunction used to determine if a given graph is
///   a stapl::dynamic_graph.
/// @tparam Graph The type of the graph in question.
//////////////////////////////////////////////////////////////////////
template<typename Graph>
struct is_dynamic
{ };

//////////////////////////////////////////////////////////////////////
/// @brief @copybrief is_dynamic
/// @tparam Graph Type of the graph in question.
/// @tparam D Directedness of the graph.
/// @tparam M If the graph has multi-edges.
/// @tparam Vp Vertex property of the graph.
/// @tparam Ep Edge property of the graph.
/// @tparam PS Partitioner for the graph.
/// @tparam Map Mapper for the graph.
/// @tparam Traits Traits class for the graph.
///
/// Specialization of @ref is_dynamic for STAPL graphs with attributes.
//////////////////////////////////////////////////////////////////////
template<template<graph_attributes,graph_attributes,typename...>
         class Graph, graph_attributes D, graph_attributes M,
         typename ...OptionalParams>
struct is_dynamic <Graph<D, M, OptionalParams...> >
{
  typedef Graph<D, M, OptionalParams...> graph_t;
  typedef dynamic_graph<D, M, OptionalParams...> dynamic_graph_t;
  static const bool value
  = boost::is_base_of<dynamic_graph_t, graph_t>::value;
};

//////////////////////////////////////////////////////////////////////
/// @brief @copybrief is_dynamic
/// @tparam Graph Type of the graph in question.
/// @tparam D Directedness of the graph.
/// @tparam M If the graph has multi-edges.
/// @tparam Vp Vertex property of the graph.
/// @tparam Ep Edge property of the graph.
/// @tparam PS Partitioner for the graph.
/// @tparam Map Mapper for the graph.
/// @tparam Traits Traits class for the graph.
/// @tparam Accessor Accessor that creates a reference of the container
///         as a proxy.
///
/// Specialization of @ref is_dynamic for STAPL graph proxy with attributes.
//////////////////////////////////////////////////////////////////////
template<template<graph_attributes,graph_attributes,typename...>
         class Graph, graph_attributes D, graph_attributes M,
         typename ...OptionalParams, typename Accessor>
struct is_dynamic <proxy<Graph<D, M, OptionalParams...>, Accessor> >
{
  typedef Graph<D, M, OptionalParams...> graph_t;
  typedef dynamic_graph<D, M, OptionalParams...> dynamic_graph_t;
  static const bool value
  = boost::is_base_of<dynamic_graph_t, graph_t>::value;
};

//////////////////////////////////////////////////////////////////////
/// @brief @copybrief is_dynamic
/// @tparam Graph Type of the graph in question.
/// @tparam Vp Vertex property of the graph.
/// @tparam Ep Edge property of the graph.
/// @tparam PS Partitioner for the graph.
/// @tparam Map Mapper for the graph.
/// @tparam Traits Traits class for the graph.
///
/// Specialization of @ref is_dynamic for STAPL graphs with unknown attributes.
/// Performs checks for each combination of directedness and multiedges.
//////////////////////////////////////////////////////////////////////
template<template<typename...>
         class Graph, typename ...OptionalParams>
struct is_dynamic <Graph<OptionalParams...> >
{
  typedef Graph<OptionalParams...> graph_t;
  typedef dynamic_graph<DIRECTED, MULTIEDGES, OptionalParams...>
    dynamic_graph_t1;
  typedef dynamic_graph<UNDIRECTED, MULTIEDGES, OptionalParams...>
    dynamic_graph_t2;
  typedef dynamic_graph<DIRECTED, NONMULTIEDGES, OptionalParams...>
    dynamic_graph_t3;
  typedef dynamic_graph<UNDIRECTED, NONMULTIEDGES, OptionalParams...>
    dynamic_graph_t4;
  static const bool value
  = boost::is_base_of<dynamic_graph_t1, graph_t>::value ||
    boost::is_base_of<dynamic_graph_t2, graph_t>::value ||
    boost::is_base_of<dynamic_graph_t3, graph_t>::value ||
    boost::is_base_of<dynamic_graph_t4, graph_t>::value;
};


//////////////////////////////////////////////////////////////////////
/// @brief Function which adds vertices to the graph using the correct helper
///   class.
/// @param g View over the graph to which the vertices are added.
/// @param wf Functor which is used to add the vertices.
/// @param n The number of vertices to add.
///
/// @see av_helper
//////////////////////////////////////////////////////////////////////
template<typename GraphView, typename WF>
void add_verts_helper(GraphView& g, WF wf, size_t n)
{
  typedef typename GraphView::view_container_type graph_cont_t;
  av_helper<WF, is_dynamic<graph_cont_t>::value>(wf, n)(g);
}


} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Metafunction which extracts the type of the generated graph from
///   a graph generator type.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct extract_graph_type;

//////////////////////////////////////////////////////////////////////
/// @brief @copybrief extract_graph_type
/// @tparam Gen Type of the generator.
/// @tparam G Type of the graph.
///
/// Specialization for generators which only accept G as a parameter.
//////////////////////////////////////////////////////////////////////
template<template<typename> class Gen, typename G>
struct extract_graph_type<Gen<G> >
{
  typedef G type;
};

//////////////////////////////////////////////////////////////////////
/// @brief @copybrief extract_graph_type
/// @tparam Gen Type of the generator.
/// @tparam G Type of the graph.
/// @tparam EF Type of the edge addition functor.
/// @tparam VF Type of the vertex addition functor.
///
/// Specialization for generators which accept G, EF, and VF as parameters.
//////////////////////////////////////////////////////////////////////
template<template<typename, typename, typename> class Gen,
         typename G, typename EF, typename VF>
struct extract_graph_type<Gen<G, EF, VF> >
{
  typedef G type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Base class for all graph generators.
/// @tparam Derived Type of the derived class, used to implement static
///   polymorphism via the CRTP idiom.
//////////////////////////////////////////////////////////////////////
template<typename Derived>
class generator_base
{
  typedef typename extract_graph_type<Derived>::type graph_type;
  typedef typename graph_type::view_container_type   graph_cont_type;
  typedef Derived                                    derived_type;

  graph_type m_graph;
  size_t     m_num_vertices;

  //////////////////////////////////////////////////////////////////////
  /// @brief Cast this object to the derived type.
  /// @return Reference to this class interpreted as the derived type.
  //////////////////////////////////////////////////////////////////////
  derived_type& derived()
  {
    return static_cast<derived_type&>(*this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Cast this object to the derived type.
  /// @return Reference to this class interpreted as the derived type.
  //////////////////////////////////////////////////////////////////////
  derived_type const& derived() const
  {
    return static_cast<derived_type const&>(*this);
  }

protected:
  graph_type& graph()
  {
    return m_graph;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function which is called to add vertices to the graph using the
  ///   default functor.
  //////////////////////////////////////////////////////////////////////
  void add_vertices()
  {
    detail::add_verts_helper(this->graph(), detail::populate_vertices(),
                             m_num_vertices);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function which is called to add vertices to the graph using the
  ///   provided vertex addition functor.
  /// @param vf Functor which is used to add vertices.
  //////////////////////////////////////////////////////////////////////
  template<typename VF>
  void add_vertices(VF const& vf)
  {
    detail::add_verts_helper(this->graph(), vf, m_num_vertices);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function which is called to add edges to the graph using the
  ///   provided edge addition functor.
  /// @param ef Functor which is used to add edges.
  //////////////////////////////////////////////////////////////////////
  template<typename EF>
  void add_edges(EF const& ef)
  {
    map_func(ef, this->graph(), make_repeat_view(this->graph()));
  }

public:
  // if no graph is provided, we generate default vertices from [0, N)
  // with default properties.
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a generator, as well as a new graph of size n.
  /// @param n Size of the new graph.
  //////////////////////////////////////////////////////////////////////
  generator_base(size_t n)
    : m_graph(new graph_cont_type(n)),
      m_num_vertices(n)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param g View over the generated graph.
  /// @param n Size of the graph.
  //////////////////////////////////////////////////////////////////////
  generator_base(graph_type& g, size_t n)
    : m_graph(g),
      m_num_vertices(n)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Calls @ref add_vertices and @ref add_edges on the derived class.
  /// @return A view over the generated graph.
  //////////////////////////////////////////////////////////////////////
  graph_type operator()()
  {
    size_t graph_sz = m_graph.size();
    rmi_fence();  // Needed because size is non-collective.
    if (graph_sz == 0) {
      derived().add_vertices();
      rmi_fence();  // Needed because of comm. generated during vertex-addition.
    }

    stapl_assert(m_graph.size() == m_num_vertices,
                 "Graph does not have required number of vertices");

    if (m_graph.version() != m_graph.container().version())
      m_graph = graph_type(this->graph().container());
    derived().add_edges();
    rmi_fence();    // Needed because of comm. generated during edge-addition.
    return this->graph();
  }

  void define_type(typer& t)
  {
    t.member(m_graph);
    t.member(m_num_vertices);
  }

}; // class generator_base


//////////////////////////////////////////////////////////////////////
/// @brief Class for generating random numbers.
///
/// Generators that need to produce random numbers in a thread-safe
/// way should inherit from this class and call this->rand(...) to
/// get random numbers.
//////////////////////////////////////////////////////////////////////
struct rand_gen
{
  boost::random::mt19937 m_rng;
  typedef boost::random::uniform_int_distribution<size_t> rng_dist_t;

  //////////////////////////////////////////////////////////////////////
  /// @param seed The seed for random-number generation.
  //////////////////////////////////////////////////////////////////////
  rand_gen(unsigned int seed = get_location_id())
    : m_rng(seed)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Generates a random number in the range
  /// [0, numeric_limits<size_t>::max()).
  //////////////////////////////////////////////////////////////////////
  size_t rand(void)
  { return rng_dist_t()(m_rng); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Generates a random number in the range
  /// [0, max).
  /// @param max The maximum value of the output random number (exclusive).
  //////////////////////////////////////////////////////////////////////
  size_t rand(size_t max)
  { return rng_dist_t(0, max-1)(m_rng); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Generates a random number in the range
  /// [min, max).
  /// @param min The minimum value of the output random number.
  /// @param max The maximum value of the output random number (exclusive).
  //////////////////////////////////////////////////////////////////////
  size_t rand(size_t min, size_t max)
  { return rng_dist_t(min, max-1)(m_rng); }

  void define_type(typer& t)
  { t.member(m_rng); }
};

} // namespace generators

} // namespace stapl

#endif

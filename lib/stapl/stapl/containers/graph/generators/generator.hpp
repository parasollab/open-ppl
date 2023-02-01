/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_GENERATOR_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_GENERATOR_HPP

#include <stapl/containers/graph/generators/generator_base.hpp>

namespace stapl {

namespace generators {

//////////////////////////////////////////////////////////////////////
/// @brief Generator which generates a graph using the provided edge and vertex
///   functors.
/// @tparam G Type of the view over the generated graph.
/// @tparam EF Type of the edge generation functor.
/// @tparam VF Type of the vertex generation functor.
///
/// Generates a graph with N vertices. Vertices are generated using the provided
/// functor, or are numbered [0,N) with a default property otherwise. Edges are
/// added using the provided functor.
//////////////////////////////////////////////////////////////////////
template<typename G, typename EF, typename VF = detail::populate_vertices>
struct generator
  : public generator_base<generator<G, EF, VF> >
{
  typedef generator_base<generator<G, EF, VF> > base_type;

protected:
  EF m_edge_gen;
  VF m_vertex_gen;

public:
  //////////////////////////////////////////////////////////////////////
  /// @param g View of the graph to generate.
  /// @param num_vertices The number of vertices in the graph.
  /// @param edge_gen The edge generator.
  /// @param vertex_gen The vertex generator.
  //////////////////////////////////////////////////////////////////////
  generator(G& g, size_t num_vertices,
            EF const& edge_gen, VF const& vertex_gen = VF())
    : base_type(g, num_vertices), m_edge_gen(edge_gen), m_vertex_gen(vertex_gen)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @param num_vertices The number of vertices in the graph.
  /// @param edge_gen The edge generator.
  /// @param vertex_gen The vertex generator.
  //////////////////////////////////////////////////////////////////////
  generator(size_t num_vertices,
            EF const& edge_gen, VF const& vertex_gen = VF())
    : base_type(num_vertices), m_edge_gen(edge_gen), m_vertex_gen(vertex_gen)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function which is called to add vertices.
  //////////////////////////////////////////////////////////////////////
  void add_vertices()
  {
    base_type::add_vertices(m_vertex_gen);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Function which is called to add edges.
  //////////////////////////////////////////////////////////////////////
  void add_edges()
  {
    base_type::add_edges(m_edge_gen);
  }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
    t.member(m_edge_gen);
    t.member(m_vertex_gen);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function which creates a graph generator with the given parameters.
/// @param g A view over the graph to generate.
/// @param num_vertices Number of vertices in the generated graph.
/// @param edge_gen A functor which is called to generate the edges.
/// @param vertex_gen A functor which is called to generate the vertices.
/// @return A generator object which will generate the desired graph upon
///   invocation.
///
/// This function mutates the input graph.
/// @todo As this is a mutating function (adds vertices and/or edges),
/// the view is passed by & (vs. const&). However, this may require
/// users to write an additional line declaring the view, vs. the
/// convenience of creating a view inline with calling the generator.
//////////////////////////////////////////////////////////////////////
template <typename GraphView, typename EF, typename VF>
generator<GraphView, EF, VF>
make_generator(GraphView& g, size_t num_vertices,
               EF const& edge_gen, VF const& vertex_gen)
{
  return generator<GraphView, EF, VF>(g, num_vertices, edge_gen, vertex_gen);
}

//////////////////////////////////////////////////////////////////////
/// @brief @copybrief make_generator
/// @param num_vertices Number of vertices in the generated graph.
/// @param edge_gen A functor which is called to generate the edges.
/// @param vertex_gen A functor which is called to generate the vertices.
/// @return A generator object which will generate the desired graph upon
///   invocation.
///
/// The generator object will return a view which owns its underlying container.
/// @see make_generator
//////////////////////////////////////////////////////////////////////
template <typename GraphView, typename EF, typename VF>
generator<GraphView, EF, VF>
make_generator(size_t num_vertices, EF const& edge_gen, VF const& vertex_gen)
{
  return generator<GraphView, EF, VF>(num_vertices, edge_gen, vertex_gen);
}

//////////////////////////////////////////////////////////////////////
/// @brief Function which creates a graph generator with the given parameters.
/// @param g A view over the graph to generate.
/// @param num_vertices Number of vertices in the generated graph.
/// @param edge_gen A functor which is called to generate the edges.
/// @return A generator object which will generate the desired graph upon
///   invocation.
///
/// This function mutates the input graph.
/// @todo As this is a mutating function (adds vertices and/or edges),
/// the view is passed by & (vs. const&). However, this may require
/// users to write an additional line declaring the view, vs. the
/// convenience of creating a view inline with calling the generator.
//////////////////////////////////////////////////////////////////////
template <typename GraphView, typename EF>
generator<GraphView, EF>
make_generator(GraphView& g, size_t num_vertices, EF const& edge_gen)
{
  return generator<GraphView, EF>(g, num_vertices, edge_gen);
}

//////////////////////////////////////////////////////////////////////
/// @brief @copybrief make_generator
/// @param num_vertices Number of vertices in the generated graph.
/// @param edge_gen A functor which is called to generate the edges.
/// @return A generator object which will generate the desired graph upon
///   invocation.
///
/// The generator object will return a view which owns its underlying container.
/// @see make_generator
//////////////////////////////////////////////////////////////////////
template <typename GraphView, typename EF>
generator<GraphView, EF>
make_generator(size_t num_vertices, EF const& edge_gen)
{
  return generator<GraphView, EF>(num_vertices, edge_gen);
}

} // namespace generators

} // namespace stapl

#endif

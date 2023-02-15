/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_TORUS_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_TORUS_HPP

#include <stapl/containers/graph/generators/generator.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which connects vertices to form a torus of the given
///   dimensions.
/// @see make_torus
//////////////////////////////////////////////////////////////////////
struct torus_neighbors
{
  size_t m_x, m_y;
  bool m_bidirectional;

  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param x,y Dimensions of the mesh.
  /// @param bidirectional True to add back-edges in a directed graph.
  //////////////////////////////////////////////////////////////////////
  torus_neighbors(size_t x, size_t y, bool bidirectional)
    : m_x(x), m_y(y), m_bidirectional(bidirectional)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    size_t row_st = (v.descriptor()/m_x)*m_x;
    size_t x_neighbour = (((v.descriptor()-(row_st)+1)%m_x) + (row_st));
    size_t y_neighbour = (v.descriptor()+m_x)%(m_x*m_y);

    view.add_edge_async(v.descriptor(), x_neighbour);
    view.add_edge_async(v.descriptor(), y_neighbour);

    if (view.is_directed() && m_bidirectional) {
      view.add_edge_async(x_neighbour, v.descriptor());
      view.add_edge_async(y_neighbour, v.descriptor());
    }
  }

  void define_type(typer& t)
  {
    t.member(m_x);
    t.member(m_y);
    t.member(m_bidirectional);
  }
};

}


//////////////////////////////////////////////////////////////////////
/// @brief Generates an x-by-y torus.
///
/// The generated torus will have x vertices in the horizontal direction and
/// y vertices in the vertical direction. This is distributed n/p in the
/// y-direction.
///
/// This function mutates the input graph.
///
/// @param g A view over the graph to generate.
/// @param nx Size of the x-dimension of the torus.
/// @param ny Size of the y-dimension of the torus.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_torus(GraphView& g, size_t nx, size_t ny,
                     bool bidirectional=true)
{
  typedef typename detail::torus_neighbors ef_t;
  return make_generator<GraphView, ef_t>(g, nx*ny, ef_t(nx, ny,
                                                        bidirectional))();
}

//////////////////////////////////////////////////////////////////////
/// @brief Generates an x-by-y torus.
///
/// The generated torus will have x vertices in the horizontal direction and
/// y vertices in the vertical direction. This is distributed n/p in the
/// y-direction.
///
/// The returned view owns its underlying container.
///
/// @param nx Size of the x-dimension of the torus.
/// @param ny Size of the y-dimension of the torus.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return A view over the generated graph.
///
/// @b Example
/// @snippet torus.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_torus(size_t nx, size_t ny, bool bidirectional=true)
{
  typedef typename detail::torus_neighbors ef_t;
  return make_generator<GraphView, ef_t>(nx*ny, ef_t(nx, ny, bidirectional))();
}


} // namespace generators

} // namespace stapl

#endif

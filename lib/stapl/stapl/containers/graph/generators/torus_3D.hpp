/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_TORUS_3D_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_TORUS_3D_HPP

#include <stapl/containers/graph/generators/generator.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which connects vertices to form a 3D-torus of the given
///   dimensions.
/// @see make_torus_3D
//////////////////////////////////////////////////////////////////////
struct torus_neighbors_3D
{
  size_t m_x, m_y, m_z;
  bool m_bidirectional;

  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param x,y,z Dimensions of the mesh.
  /// @param bidirectional True to add back-edges in a directed graph.
  //////////////////////////////////////////////////////////////////////
  torus_neighbors_3D(size_t x, size_t y, size_t z, bool bidirectional)
    : m_x(x), m_y(y), m_z(z), m_bidirectional(bidirectional)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    size_t row_st  = (v.descriptor() / m_x) * m_x;
    size_t face_st = (v.descriptor() / (m_x*m_y)) * (m_x*m_y);
    size_t cube_st = (v.descriptor() / (m_x*m_y*m_z)) * (m_x*m_y*m_z);

    const size_t x_neighbour
      = ((v.descriptor() - row_st + 1) % m_x) + row_st;

    const size_t y_neighbour
      = ((v.descriptor() - face_st + m_x) % (m_x*m_y)) + face_st;

    const size_t z_neighbour
      = ((v.descriptor() - cube_st + (m_x*m_y)) % (m_x*m_y*m_z)) + cube_st;

    view.add_edge_async(v.descriptor(), x_neighbour);
    view.add_edge_async(v.descriptor(), y_neighbour);
    view.add_edge_async(v.descriptor(), z_neighbour);

    if (view.is_directed() && m_bidirectional) {
      view.add_edge_async(x_neighbour, v.descriptor());
      view.add_edge_async(y_neighbour, v.descriptor());
      view.add_edge_async(z_neighbour, v.descriptor());
    }
  }

  void define_type(typer& t)
  {
    t.member(m_x);
    t.member(m_y);
    t.member(m_z);
    t.member(m_bidirectional);
  }
};

}


//////////////////////////////////////////////////////////////////////
/// @brief Generates an x-by-y-by-z 3D torus.
///
/// The generated torus will have x vertices in the horizontal direction,
/// y vertices in the vertical direction, and z vertices in the in-out
/// direction. This is distributed n/p in the z-direction, where n = x*y*z.
///
/// This function mutates the input graph.
///
/// @param g A view over the graph to generate.
/// @param nx Size of the x-dimension of the torus.
/// @param ny Size of the y-dimension of the torus.
/// @param nz Size of the z-dimension of the torus.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_torus_3D(GraphView& g, size_t nx, size_t ny, size_t nz,
                        bool bidirectional=true)
{
  typedef typename detail::torus_neighbors_3D ef_t;
  return make_generator<GraphView, ef_t>(g, nx*ny*nz, ef_t(nx, ny, nz,
                                                           bidirectional))();
}


//////////////////////////////////////////////////////////////////////
/// @brief Generates an x-by-y-by-z 3D torus.
///
/// The generated torus will have x vertices in the horizontal direction,
/// y vertices in the vertical direction, and z vertices in the in-out
/// direction. This is distributed n/p in the z-direction, where n = x*y*z.
///
/// The returned view owns its underlying container.
///
/// @param nx Size of the x-dimension of the torus.
/// @param ny Size of the y-dimension of the torus.
/// @param nz Size of the z-dimension of the torus.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return A view over the generated graph.
///
/// @b Example
/// @snippet torus_3D.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_torus_3D(size_t nx, size_t ny, size_t nz,
                        bool bidirectional=true)
{
  typedef typename detail::torus_neighbors_3D ef_t;
  return make_generator<GraphView, ef_t>(nx*ny*nz, ef_t(nx, ny, nz,
                                                        bidirectional))();
}

} // namespace generators

} // namespace stapl

#endif

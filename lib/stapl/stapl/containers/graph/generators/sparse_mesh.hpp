/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_SPARSE_MESH_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_SPARSE_MESH_HPP

#include <stapl/containers/graph/generators/generator.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which adds edges with the given probability to form a mesh
///   of the given dimensions.
/// @see make_sparse_mesh
//////////////////////////////////////////////////////////////////////
struct sparse_mesh_neighbors
  : rand_gen
{
  size_t m_x, m_y;
  double m_p;
  bool m_bidirectional;

  //////////////////////////////////////////////////////////////////////
  /// @param x,y Dimensions of the mesh.
  /// @param p Probability of adding a given edge in the mesh.
  /// @param bidirectional True to add back-edges in a directed graph.
  /// @param seed The seed for random-number generation.
  //////////////////////////////////////////////////////////////////////
  sparse_mesh_neighbors(size_t x, size_t y, double p, bool bidirectional,
                        unsigned int seed = get_location_id())
    : rand_gen(seed), m_x(x), m_y(y), m_p(p), m_bidirectional(bidirectional)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    size_t row_st = (v.descriptor()/m_x)*m_x;
    size_t x_neighbour = ((v.descriptor()-(row_st)+1) + row_st);
    size_t y_neighbour = (v.descriptor()+m_x);

    bool add_x = (this->rand(100) <= m_p*100);
    bool add_y = (this->rand(100) <= m_p*100);

    if (x_neighbour < row_st+m_x && add_x)
      view.add_edge_async(v.descriptor(), x_neighbour);
    if (y_neighbour < m_x*m_y && add_y)
      view.add_edge_async(v.descriptor(), y_neighbour);

    if (view.is_directed() && m_bidirectional) {
      if (x_neighbour < row_st+m_x && add_x)
        view.add_edge_async(x_neighbour, v.descriptor());
      if (y_neighbour < m_x*m_y && add_y)
        view.add_edge_async(y_neighbour, v.descriptor());
    }
  }

  void define_type(typer& t)
  {
    t.member(m_x);
    t.member(m_y);
    t.member(m_p);
    t.member(m_bidirectional);
  }
};

}


//////////////////////////////////////////////////////////////////////
/// @brief Generates an x-by-y sparse mesh.
///
/// The generated sparse mesh will contain x vertices in the horizontal direction and
/// y vertices in the vertical direction. This is distributed n/p in the
/// y-direction.
///
/// Each edge is added with a probability of p. Thus, p = 1 generates a complete
/// mesh, and p = 0.5 generates a mesh with 50% of the edges of a complete mesh.
///
/// This function mutates the input graph.
///
/// @param g A view over the graph to generate.
/// @param nx Size of the x-dimension of the sparse mesh.
/// @param ny Size of the y-dimension of the sparse mesh.
/// @param p Probability of adding a particular edge to the mesh.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_sparse_mesh(GraphView& g, size_t nx, size_t ny,
                           double p = 0.3, bool bidirectional=true)
{
  typedef typename detail::sparse_mesh_neighbors ef_t;
  return make_generator<GraphView, ef_t>(g, nx*ny, ef_t(nx, ny, p,
                                                        bidirectional))();
}

//////////////////////////////////////////////////////////////////////
/// @brief Generates an x-by-y sparse mesh.
///
/// The generated sparse mesh will contain x vertices in the horizontal direction and
/// y vertices in the vertical direction. This is distributed n/p in the
/// y-direction.
///
/// The returned view owns its underlying container.
///
/// Each edge is added with a probability of p. Thus, p = 1 generates a complete
/// mesh, and p = 0.5 generates a mesh with 50% of the edges of a complete mesh.
/// @param nx Size of the x-dimension of the sparse mesh.
/// @param ny Size of the y-dimension of the sparse mesh.
/// @param p Probability of adding a particular edge to the mesh.
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return A view over the generated graph.
///
/// @b Example
/// @snippet sparse_mesh.cc Example
/// @addtogroup SGLReferenceGenerators
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
GraphView make_sparse_mesh(size_t nx, size_t ny,
                           double p = 0.3, bool bidirectional=true)
{
  typedef typename detail::sparse_mesh_neighbors ef_t;
  return make_generator<GraphView, ef_t>(nx*ny, ef_t(nx, ny, p,
                                                     bidirectional))();
}


} // namespace generators

} // namespace stapl

#endif

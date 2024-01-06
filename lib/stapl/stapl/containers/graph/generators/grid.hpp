/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_GENERATORS_GRID_HPP
#define STAPL_CONTAINERS_GRAPH_GENERATORS_GRID_HPP

#include <stapl/containers/graph/generators/generator.hpp>
#include <stapl/views/metadata/projection/geometry.hpp>

namespace stapl {

namespace generators {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor which adds edges to form a grid of the given dimensions.
/// @see make_grid
//////////////////////////////////////////////////////////////////////
template<int D>
struct grid_neighbors
{
  geometry_impl::grid_generator<D> m_generator;
  bool                             m_bidirectional;

  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param x,y Dimensions of the grid.
  /// @param bidirectional True to add back-edges in a directed graph.
  //////////////////////////////////////////////////////////////////////
  grid_neighbors(std::array<std::size_t, D> dims, bool bidirectional)
    : m_generator(dims), m_bidirectional(bidirectional)
  { }

  template<typename Vertex, typename Graph>
  void operator()(Vertex v, Graph& view)
  {
    typedef typename Vertex::vertex_descriptor descriptor_type;

    m_generator.edges_for(v.descriptor(),
      [&](descriptor_type const& s, descriptor_type const& t) {

      view.add_edge_async(s, t);

       if (m_bidirectional && view.is_directed())
        view.add_edge_async(t, s);
    });
  }

  void define_type(typer& t)
  {
    t.member(m_generator);
    t.member(m_bidirectional);
  }
};

}


//////////////////////////////////////////////////////////////////////
/// @brief Generates a multidimensional grid graph.
///
/// The generated graph will be a hyperrectangular grid in n-dimensions.
/// In 2D, it will create a 2D mesh. In 3D, it will create a cube and
/// so on.
///
/// This function mutates the input graph.
///
/// @param g A view over the graph to generate.
/// @param dims An array of sizes in each dimension
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return The original view, now containing the generated graph.
//////////////////////////////////////////////////////////////////////
template <typename GraphView, unsigned long D>
GraphView make_grid(GraphView& g, std::array<std::size_t, D> dims,
                    bool bidirectional=true)
{
  typedef detail::grid_neighbors<D> ef_t;
  const std::size_t size =
    std::accumulate(
      dims.begin(), dims.end(), 1, std::multiplies<std::size_t>()
    );

  return make_generator<GraphView, ef_t>(g, size,
                                         ef_t(dims, bidirectional))();
}

//////////////////////////////////////////////////////////////////////
/// @brief Generates a multidimensional grid graph.
///
/// The generated graph will be a hyperrectangular grid in n-dimensions.
/// In 2D, it will create a 2D mesh. In 3D, it will create a cube and
/// so on.
/// The returned view owns its underlying container.
///
/// @param dims An array of sizes in each dimension
/// @param bidirectional True to add back-edges in a directed graph, false
///   for forward edges only.
/// @return A view over the generated graph.
///
/// @b Example
/// @snippet grid.cc Example
//////////////////////////////////////////////////////////////////////
template <typename GraphView, unsigned long D>
GraphView make_grid(std::array<std::size_t, D> dims, bool bidirectional=true)
{
  typedef detail::grid_neighbors<D> ef_t;
  const std::size_t size =
    std::accumulate(
      dims.begin(), dims.end(), 1, std::multiplies<std::size_t>()
    );

  return make_generator<GraphView, ef_t>(size, ef_t(dims, bidirectional))();
}


} // namespace generators

} // namespace stapl

#endif

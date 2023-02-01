/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_PROJECTION_GEOMETRY_HPP
#define STAPL_VIEWS_METADATA_PROJECTION_GEOMETRY_HPP

#include <stapl/containers/sequential/graph/graph.h>
#include <stapl/utility/do_once.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/pack_ops.hpp>

#include <stapl/containers/type_traits/dimension_traits.hpp>
#include <stapl/domains/indexed.hpp>

namespace stapl {

namespace geometry_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Helper class to determine if two d-dimensional hyperrectangles
///        are mergeable to form a new hyperrectangle.
///
/// @tparam D The domain type for each hyperrectangle
//////////////////////////////////////////////////////////////////////
template<typename D, typename Indices = make_index_sequence<
           dimension_traits<typename D::gid_type>::type::value>>
class is_mergeable;

template<typename D, std::size_t... Dims>
class is_mergeable<D, index_sequence<Dims...>>
{
  //////////////////////////////////////////////////////////////////////
  /// @brief Determine if two hyperrectangles touch in the k'th dimension
  ///        by seeing if either of them are 1 past the other
  //////////////////////////////////////////////////////////////////////
  template<int k>
  static bool touching(D const& r1, D const& r2)
  {
    return get<k>(r1.last())+1 == get<k>(r2.first()) or
           get<k>(r1.first()) == get<k>(r2.last())+1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Check if the i'th dimension of both hyperrectangles are the
  ///        same. If i is k, then we ignore it.
  //////////////////////////////////////////////////////////////////////
  template<int k, int i>
  static bool same_dimension_bounds(D const& r1, D const& r2)
  {
    return i == k ? true : get<i>(r1.first()) == get<i>(r2.first()) and
                           get<i>(r1.last()) == get<i>(r2.last());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief See if all of the dimensions have the same bounds for both
  ///        hyperrectangles except for the k'th dimension
  //////////////////////////////////////////////////////////////////////
  template<int k>
  static bool same_hyperplane_volume(D const& r1, D const& r2)
  {
    return pack_ops::functional::and_(
      same_dimension_bounds<k, Dims>(r1, r2)...
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Determine if two hyperrectangles are mergeable in the k'th
  ///        dimension
  //////////////////////////////////////////////////////////////////////
  template<int k>
  static bool mergeable_in(D const& r1, D const& r2)
  {
    // they can be merged if they are touching in the k'th dimension
    // and the other dimensions (besides k) are the same
    return touching<k>(r1, r2) and same_hyperplane_volume<k>(r1, r2);
  }

public:
  static bool apply(D const& r1, D const& r2)
  {
    // the hyperrectangles are mergeable if they are mergeable
    // in at least one dimension
    return pack_ops::functional::or_(mergeable_in<Dims>(r1, r2)...);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Generates a d-dimensional grid sequential graph. This is
///        also used by the parallel graph generator @see make_grid.
///
/// @tparam D The number of dimensions of the grid
//////////////////////////////////////////////////////////////////////
template<int D>
struct grid_generator
{
  std::array<std::size_t, D> m_sizes;

  template<typename Dims>
  grid_generator(Dims const& dims)
   : m_sizes(homogeneous_tuple_to_array<Dims>::apply(dims))
  {
    std::reverse(std::begin(m_sizes), std::end(m_sizes));
  }

  grid_generator(std::array<std::size_t, D> const& dims)
   : m_sizes(dims)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Computes the edges of a given vertex based on its
  ///        descriptor and calls a function with the source and target
  ///        of that edge.
  ///
  /// @param i The descriptor of the vertex
  /// @param f A binary function that will be invoked for each edge of i
  //////////////////////////////////////////////////////////////////////
  template<typename Descriptor, typename F>
  void edges_for(Descriptor i, F&& f) const
  {
    using std::begin;
    using std::end;

    std::array<std::size_t, D> starts;
    std::array<std::size_t, D> products;

    std::fill(begin(products), end(products), 0);
    std::partial_sum(begin(m_sizes), end(m_sizes),
      begin(products), std::multiplies<std::size_t>()
    );
    std::copy(begin(m_sizes), end(m_sizes), begin(starts));

    // calculate the offset of this dimension
    std::transform(begin(products), end(products), begin(starts),
      [&](std::size_t product) {
        return (i / product) * product;
      });

    // Generalization of the 3D formula:
    // x: ((v.descriptor() - row_st + 1) % m_x) + row_st;
    // y: ((v.descriptor() - face_st + m_x) % (m_x*m_y)) + face_st;
    // z: ((v.descriptor() - cube_st + (m_x*m_y)) % (m_x*m_y*m_z)) + cube_st;
    for (int d = 0; d < D; ++d)
    {
      const std::size_t last_dimension_offset = d == 0 ? 1 : products[d-1];
      const std::size_t hyperrplane_offset = i-starts[d]+last_dimension_offset;
      const std::size_t neighbor =
        (hyperrplane_offset % products[d]) + starts[d];

      if (neighbor > i)
        f(i, neighbor);
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a sequential d-dimensional grid graph.
  //////////////////////////////////////////////////////////////////////
  template<typename Graph>
  Graph build() const
  {
    const std::size_t size =
      std::accumulate(
        m_sizes.begin(), m_sizes.end(), 1, std::multiplies<std::size_t>()
      );

    Graph g(size);

    for (std::size_t i = 0; i < size; ++i)
      this->edges_for(i, [&](std::size_t s, std::size_t t) {
        g.add_edge(s, t);
      });

    return g;
  }

  void define_type(typer& t)
  {
    t.member(m_sizes);
  }
};

} // namespace geometry_impl

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_PROJECTION_GEOMETRY_HPP

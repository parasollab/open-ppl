/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_SKELETONS_FUNCTIONAL_TRANSPOSE_2D_HPP
#define STAPL_SKELETONS_FUNCTIONAL_TRANSPOSE_2D_HPP

#include <vector>
#include <stapl/skeletons/functional/map.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/functional/alltoall.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/utility/lightweight_multiarray.hpp>
#include <stapl/skeletons/utility/tags.hpp>
#include "flows/transpose_2d.hpp"

namespace stapl {
namespace skeletons {
namespace skeletons_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Prepares the input for the global exchange (@c alltoall)
/// phase of a 2D matrix transpose. During this step, the part of input
/// that is received by this workfunction is divided into chunks and
/// passed to the global exchange phase.
///
/// This workfunction can be used to transpose a 2D matrix that is
/// partitioned along one dimension. A different method should be used
/// for 2D partitioned matrices.
///
/// @tparam T           type of the input values.
/// @tparam Partitioner the type of the partitioner used for the output.
//////////////////////////////////////////////////////////////////////
template <typename T, typename Partitioner>
struct prepare_transpose_2d
{
  using result_type = std::vector<lightweight_multiarray<T, 2>>;

private:
  using partitioner_t = typename std::decay<Partitioner>::type ;
  std::size_t numparts_x, numparts_y;

public:
  template <typename Dims>
  prepare_transpose_2d(Dims&& dims)
  {
    std::tie(numparts_x, numparts_y) = dims;
  }

  template <typename U0>
  result_type operator()(U0&& u0)
  {
    result_type r;
    // divide the input to (n/p x m/p) chunks
    partitioner_t p(u0.domain(), make_tuple(numparts_y, numparts_x));

    using index_t = decltype(p.domain().first());
    using inner_index_t = decltype(p[std::declval<index_t>()].first());

    domain_map(p.domain(), [&](index_t i){
      std::size_t chunk_nx, chunk_ny, xoffset, yoffset;
      std::tie(chunk_nx, chunk_ny) = p[i].dimensions();
      std::tie(xoffset, yoffset) = p[i].first();

      lightweight_multiarray<T, 2> v(chunk_ny, chunk_nx);

      domain_map(p[i], [&](inner_index_t j) {
        std::size_t x, y;
        std::tie(x, y) = j;
        v[make_tuple(y-yoffset, x-xoffset)] = u0[j];
      });

      r.push_back(v);
    });

    return r;
  }

  void define_type(typer& t)
  {
    t.member(numparts_x);
    t.member(numparts_y);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Puts the value received after the global exchange phase of
/// a 2D transpose in the given output.
///
/// This workfunction can be used to transpose a 2D matrix that is
/// partitioned along one dimension. A different method should be used
/// for 2D partitioned matrices.
//////////////////////////////////////////////////////////////////////
struct finish_transpose_2d
{
  using result_type = void;

  template <typename Chunks, typename U1>
  void operator()(Chunks&& chunks, U1&& u1)
  {
    auto dom = u1.domain();
    std::size_t dimx, dimy;
    std::tie(dimx, dimy) = dom.dimensions();

    std::size_t u1_offset_x, u1_offset_y;
    std::tie(u1_offset_x, u1_offset_y) = dom.first();
    using index_type = decltype(dom.first());

    for (auto const& chunk : chunks){
      std::size_t chunk_offset_x = u1_offset_x;
      std::size_t chunk_offset_y = u1_offset_y;
      std::size_t chunk_dimension_x, chunk_dimension_y;
      index_type dims = chunk.dimensions();
      std::tie(chunk_dimension_x, chunk_dimension_y) = dims;

      for (std::size_t i = 0; i < chunk_dimension_x; ++i) {
        for (std::size_t j = 0; j < chunk_dimension_y; ++j) {
          u1[index_type(i + chunk_offset_x,
                        j + chunk_offset_y)] = chunk[index_type(i, j)];
        }
      }
      if (dimx >= dimy) {
        u1_offset_x += chunk_dimension_x;
      }
      else {
        u1_offset_y += chunk_dimension_y;
      }
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief This class abstracts the semantics of a transpose_2d skeleton
/// by exposing only the necessary information in its representation.
///
/// This abstraction not only makes the reconstruction of a
/// transpose_2d skeleton easier, but also provides access to the
/// underlying operations of the enclosed reduction operation. Furthermore,
/// it reduces the symbol size for a transpose_2d skeleton, hence, reducing
/// the total compilation time.
///
/// @tparam T           type of the elements in the matrix.
/// @tparam Partition   the dimensions of the domain of this partitioner
///                     are used to determine the parts of the matrix to
///                     be exchanged.
/// @ingroup skeletonsFunctionalInternal
//////////////////////////////////////////////////////////////////////
template<typename T, typename Partition>
struct transpose_2d
  : public decltype(
             skeletons::compose<flows::transpose_2d>(
               skeletons::map(
                 prepare_transpose_2d<T, Partition>(
                   std::declval<Partition>().domain().dimensions())),
                 skeletons::alltoall<
                   lightweight_multiarray<T, 2>, tags::flat>(),
                 skeletons::zip<2>(finish_transpose_2d()))
             )
{
  using skeleton_tag_type = tags::transpose_2d;
  using base_type = decltype(
                      skeletons::compose<flows::transpose_2d>(
                        skeletons::map(
                          prepare_transpose_2d<T, Partition>(
                            std::declval<Partition>().domain().dimensions())),
                          skeletons::alltoall<
                            lightweight_multiarray<T, 2>, tags::flat>(),
                          skeletons::zip<2>(finish_transpose_2d())));

  transpose_2d(Partition const& output_partition)
    : base_type(
        skeletons::compose<flows::transpose_2d>(
          skeletons::map(prepare_transpose_2d<T, Partition>(
                         output_partition.domain().dimensions())),
          skeletons::alltoall<lightweight_multiarray<T, 2>,
                   skeletons::tags::flat>(),
          skeletons::zip<2>(finish_transpose_2d()))
      )
  { }

  void define_type(typer& t)
  {
    t.base<base_type>(*this);
  }
};

} // namespace skeletons_impl

namespace result_of {

template<typename T,
         typename Partition>
using transpose_2d = skeletons_impl::transpose_2d<
                       T, typename std::decay<Partition>::type>;

} // namespace result_of


//////////////////////////////////////////////////////////////////////
/// @brief Transposes a 1D-decomposed 2D multiarray.
///
/// @tparam T         elements type.
/// @param  Partition the dimensions of the domain of this partitioner
///                   are used to determine the parts of the matrix to
///                   be exchanged.
///
/// @TODO This skeletons currently supports matrices with 1xp
/// decompositions and can transpose square matrices.
//////////////////////////////////////////////////////////////////////
template <typename T, typename Partition>
result_of::transpose_2d<T, Partition>
transpose_2d(Partition&& partition)
{
  return result_of::transpose_2d<T, Partition>(
           std::forward<Partition>(partition));
}

} // namespace skeletons
} // namespace stapl

#endif // STAPL_SKELETONS_FUNCTIONAL_TRANSPOSE_2D_HPP
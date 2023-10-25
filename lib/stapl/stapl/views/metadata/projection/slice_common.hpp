/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

//////////////////////////////////////////////////////////////////////
/// @file
/// Common functionality used by both @ref slices_projection and
/// @ref sliced_projection.
//////////////////////////////////////////////////////////////////////

#ifndef STAPL_VIEWS_METADATA_SLICE_COMMON_HPP
#define STAPL_VIEWS_METADATA_SLICE_COMMON_HPP

#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/utility/tuple/apply.hpp>
#include <stapl/utility/tuple/discard.hpp>
#include <stapl/utility/pack_ops.hpp>

namespace stapl {

namespace metadata {

namespace slice_common_impl {

// Test if a whole slice is contained in a single partition
struct is_single_partition_unpacked
{
  template <typename Dim0, typename... Dims>
  Dim0 operator() (Dim0 dim0, Dims... dims) const
  {
    return pack_ops::functional::and_(dim0 == 1, (dims == 1)...);
  }
};

template<typename T>
bool is_single_partition(T slice_part_dims)
{
  return slice_part_dims == 1;
}

template<typename... Ts>
bool is_single_partition(tuple<Ts...> const& slice_part_dims)
{
  return tuple_ops::apply( is_single_partition_unpacked(), slice_part_dims );
}

} // namespace slice_common_impl

//////////////////////////////////////////////////////////////////////
/// @brief Check if each slice is fully contained in a single partition.
/// @param part_dims_orig Number of partitions in each dimension of the
///        original container being sliced.
//////////////////////////////////////////////////////////////////////
template<typename ViewSlices, typename PartDim>
bool part_has_full_slice(PartDim const& part_dims_orig)
{
  return slice_common_impl::is_single_partition(
    tuple_ops::discard<ViewSlices>(part_dims_orig));
}

} // namespace metadata

} // namespace stapl

#endif // STAPL_VIEWS_METADATA_SLICE_COMMON_HPP

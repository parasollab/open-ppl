/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MULTIARRAY_METADATA_HPP
#define STAPL_CONTAINERS_MULTIARRAY_METADATA_HPP

#include <stapl/views/metadata/extraction/container_extractor_base.hpp>

namespace stapl {

namespace metadata {

//////////////////////////////////////////////////////////////////////
/// @brief Class for computing the metadata of a @ref multiarray distribution.
///
/// @tparam Distribution Type of the Distribution.
//////////////////////////////////////////////////////////////////////
template <typename Distribution,
          bool = tuple_size<tuple_ops::ensure_tuple_t<
                   typename Distribution::partition_type::index_type>>::value ==
                 tuple_size<tuple_ops::ensure_tuple_t<
                   typename Distribution::gid_type>>::value>
class multiarray_extractor;

template<typename Distribution, bool b>
struct extractor_traits<multiarray_extractor<Distribution, b>>
{
  using distribution_type = Distribution;
};


//////////////////////////////////////////////////////////////////////
/// @brief Class for computing the metadata of a @ref multiarray distribution.
///
///        This is the general case when the partition and the container
///        are of the same dimensionality.
///
/// @tparam Distribution Type of the Distribution.
//////////////////////////////////////////////////////////////////////
template<typename Distribution>
class multiarray_extractor<Distribution, true>
 : public container_extractor_base<multiarray_extractor<Distribution, true>>
{
  STAPL_IMPORT_TYPE(typename Distribution, container_manager_type)
  STAPL_IMPORT_TYPE(typename container_manager_type, base_container_type)
  STAPL_IMPORT_TYPE(typename base_container_type, cid_type)

  using index_type = cid_type;

  using size_type = typename Distribution::gid_type;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of metadata entries that should be stored
  ///
  /// @param dist The distribution of the container
  //////////////////////////////////////////////////////////////////////
  size_type num_entries(Distribution* dist) const
  {
    if (dist->partition().size() != dist->partition().domain().size())
    {
      return dist->partition().global_domain().dimensions();
    }
    return dist->partition().domain().dimensions();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the index in the metadata container that a base
  ///        container should be stored at, which is just the base
  ///        container's ID.
  ///
  /// @param dist The distribution of the container
  /// @param cid The base container ID
  //////////////////////////////////////////////////////////////////////
  index_type index_of(Distribution* dist, cid_type const& cid) const
  {
    return cid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the CID for the metadata entry at a given index in
  ///        the metadata container, which is just the index itself.
  ///
  /// @param dist The distribution of the container
  /// @param idx The index of the entry in the metadata container
  //////////////////////////////////////////////////////////////////////
  cid_type cid_of(Distribution* dist, index_type const& idx) const
  {
    return idx;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Class for computing the metadata of a @ref multiarray distribution.
///
///        This is a specialization for when the partition is of a different
///        dimensionality than the container itself, such as with a sliced
///        volumetric view-based distribution.
///
/// @tparam Distribution Type of the Distribution.
//////////////////////////////////////////////////////////////////////
template<typename Distribution>
class multiarray_extractor<Distribution, false>
 : public container_extractor_base<multiarray_extractor<Distribution, false>>
{
  STAPL_IMPORT_TYPE(typename Distribution, partition_type)

  /// The base container ID, which is of a smaller dimensionality
  /// than the container
  using cid_type = typename partition_type::index_type;
  using gid_type = typename partition_type::gid_type;

  using index_type = gid_type;
  using size_type = gid_type;

  /// The dimensions that are being partitioned
  using sliced_dimensions =
    typename partition_type::distribution_view_type::partition_dimensions_type;

  /// The sliced dimensions converted to a tuple
  using sliced =
    typename tuple_ops::from_index_sequence<sliced_dimensions>::type;

  /// The container's dimensionality
  static constexpr int num_dimensions = tuple_size<index_type>::value;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of metadata entries that should be stored
  ///
  /// @param dist The distribution of the container
  //////////////////////////////////////////////////////////////////////
  size_type num_entries(Distribution* dist) const
  {
    // get the partition's size
    cid_type original_size = dist->partition().domain().dimensions();

    // adjust the size such that it's of the same dimensionality as the
    // container, and fill in the dimensions that are not sliced with 1
    index_type adjusted_size =
      tuple_ops::expand_and_copy<num_dimensions, sliced>(
        original_size, std::integral_constant<std::size_t, 1>()
      );

    return adjusted_size;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the index in the metadata container that a base
  ///        container should be stored at. Since the base container ID
  ///        is of a lower dimension, we need to expand it and fill in
  ///        in the unused dimensions with 0.
  ///
  /// @param dist The distribution of the container
  /// @param cid The base container ID
  //////////////////////////////////////////////////////////////////////
  index_type index_of(Distribution* dist, cid_type const& cid) const
  {
    // adjust the base container ID such that it's of the same dimensionality
    // as the container, and fill in the dimensions that are not sliced with 0
    index_type idx =
      tuple_ops::expand_and_copy<num_dimensions, sliced>(
        cid, std::integral_constant<std::size_t, 0>()
      );

    return idx;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the CID for the metadata entry at a given index in
  ///        the metadata container, which is just the index itself.
  ///
  /// @param dist The distribution of the container
  /// @param idx The index of the entry in the metadata container
  //////////////////////////////////////////////////////////////////////
  index_type cid_of(Distribution* dist, index_type const& idx) const
  {
    return idx;
  }
};

} // namespace metadata

} // namespace stapl

#endif // STAPL_MULTIARRAY_METADATA_HPP

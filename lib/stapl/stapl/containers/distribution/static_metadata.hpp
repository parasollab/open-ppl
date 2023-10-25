/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_STATIC_METADATA_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_STATIC_METADATA_HPP

#include <stapl/views/metadata/extraction/container_extractor_base.hpp>

namespace stapl {

namespace metadata {

template<typename Distribution>
class static_container_extractor;


template<typename Distribution>
struct extractor_traits<static_container_extractor<Distribution>>
{
  using distribution_type = Distribution;
};


//////////////////////////////////////////////////////////////////////
/// @brief Class for computing the metadata of static containers that
/// use @ref container_manager_static that inherits from the base-container,
/// and thus, is the base-container itself. This type of container manager
/// has only one component per location.
/// @tparam Distribution Type of the Distribution.
/// @note this is different from the @ref static_array_metadata
/// primarily due to the metadata used. This class uses the
/// @ref metadata::flat_container whereas the @ref static_array_metadata
/// uses the metadata_container_wrapper.
//////////////////////////////////////////////////////////////////////
template<typename Distribution>
class static_container_extractor
 : public container_extractor_base<
     static_container_extractor<Distribution>
   >
{
  STAPL_IMPORT_TYPE(typename Distribution, container_manager_type)
  STAPL_IMPORT_TYPE(typename container_manager_type, base_container_type)
  STAPL_IMPORT_TYPE(typename base_container_type, cid_type)

  using index_type = std::size_t;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return the number of metadata entries that should be stored
  ///
  /// @param dist The distribution of the container
  //////////////////////////////////////////////////////////////////////
  std::size_t num_entries(Distribution* dist) const
  {
    return dist->num_base_containers();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the index in the metadata container that a base
  ///        container should be stored at
  ///
  /// @param dist The distribution of the container
  /// @param bc   The base container of the metadata entry
  ///
  /// @todo This may need to be refactored to take the base container and
  ///       use the base container ranking.
  //////////////////////////////////////////////////////////////////////
  index_type index_of(Distribution*, cid_type const& cid) const
  {
    return cid;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the CID for the metadata entry at a given index
  ///
  /// @param dist The distribution of the container
  /// @param bc   The index of the entry
  //////////////////////////////////////////////////////////////////////
  cid_type cid_of(Distribution*, index_type const& idx) const
  {
    return idx;
  }
};

} // namespace metadata

} // namespace stapl

#endif

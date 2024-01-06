/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_BASE_CONTAINER_METADATA_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_BASE_CONTAINER_METADATA_HPP

#include <stapl/views/metadata/container/base_container_wrapper.hpp>

namespace stapl {


//////////////////////////////////////////////////////////////////////
/// @brief Class for computing the metadata of base-containers.
/// This class contains information needed to determine the locality metadata
/// information for a base container.
/// @tparam BC Type of the base-container.
//////////////////////////////////////////////////////////////////////
template<typename BC>
struct base_container_metadata
{
  using index_type = typename BC::domain_type::index_type;
  using md_cont_type = metadata::base_container_wrapper<BC>;
  using return_type = std::pair<bool, md_cont_type*>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the metadata of the specified base-container.
  /// @param bc A pointer to the base-container.
  ///
  /// @return a pair indicating if the metadata container is static and
  ///   balance distributed and a pointer to the metadata container.
  //////////////////////////////////////////////////////////////////////
  return_type operator()(BC* bc)
  {
    return std::make_pair(false, new md_cont_type(bc));
  }
};

} // namespace stapl

#endif /* STAPL_CONTAINERS_DISTRIBUTION_BASE_CONTAINER_METADATA_HPP */

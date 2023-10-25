/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_STATIC_ARRAY_METADATA_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_STATIC_ARRAY_METADATA_HPP

#include <stapl/views/metadata/container/container_wrapper.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Class for computing the metadata of @ref static_array container that
/// uses @ref container_manager_static that inherits from the base-container,
/// and thus, is the base-container itself. This type of container manager
/// has only one component per location.
/// @tparam Distribution Type of the Distribution.
/// @note this is different from the @ref static_container_extractor primarily
/// due to the metadata used. This class uses the
/// @ref metadata_container_wrapper
/// whereas the @ref static_metadata uses the metadata::flat_container.
//////////////////////////////////////////////////////////////////////
template<typename Distribution>
struct static_array_metadata
{
  typedef metadata::container_wrapper<Distribution>    md_cont_type;
  typedef typename md_cont_type::index_type            index_type;
  typedef std::pair<bool, md_cont_type*>               return_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the metadata of specified container.
  /// @see metadata_entry.
  /// @param cont A pointer to the container.
  ///
  /// Calls the operator on the distribution of the provided container.
  ///
  /// @return a pair indicating if the metadata container is static and
  ///   balance distributed and a pointer to the metadata container.
  //////////////////////////////////////////////////////////////////////
  template <typename Container>
  return_type operator()(Container* container)
  {
    return operator()(&(container->distribution()));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the metadata of specified distribution.
  /// @see metadata_entry.
  /// @param dist A pointer to the distribution.
  ///
  /// @return a pair indicating if the metadata container is static and
  ///   balance distributed and a pointer to the metadata container.
  //////////////////////////////////////////////////////////////////////
  return_type operator()(Distribution* dist)
  {
    return std::make_pair(true, new md_cont_type(dist));
  }
};

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_STATIC_ARRAY_METADATA_HPP

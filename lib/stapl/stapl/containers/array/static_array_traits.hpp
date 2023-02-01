/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_STATIC_ARRAY_TRAITS_HPP
#define STAPL_CONTAINERS_STATIC_ARRAY_TRAITS_HPP

#include <stapl/containers/distribution/distribution.hpp>
#include <stapl/containers/distribution/operations/base.hpp>
#include <stapl/containers/distribution/operations/iterable.hpp>
#include <stapl/containers/distribution/operations/random_access.hpp>

#include <stapl/containers/distribution/container_manager/container_manager_static.hpp>
#include <stapl/containers/distribution/directory/static_registry.hpp>
#include <stapl/containers/distribution/directory/container_directory.hpp>
#include <stapl/containers/distribution/directory/manager.hpp>
#include <stapl/containers/mapping/mapper.hpp>

#include <stapl/containers/array/base_container.hpp>
#include <stapl/containers/array/base_container_traits.hpp>

#include <stapl/domains/indexed.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the static_array container. Specifies
/// customizable type parameters that could be changed on a per-container
/// basis.
/// @ingroup parrayTraits
///
/// @tparam T Type of the stored elements in the array.
/// @see static_array
////////////////////////////////////////////////////////////////////////
template<typename T, typename... OptionalNoInitParam>
struct static_array_traits
{
  using value_type      = T;
  using partition_type  = balanced_partition<indexed_domain<size_t>>;
  using mapper_type     = mapper<size_t>;
  using domain_type     = indexed_domain<size_t>;
  using index_type      = typename domain_type::index_type;
  using gid_type        = index_type;
  using manager_type    = basic_manager;

  using directory_type  =
    container_directory<
      partition_type, mapper_type, manager_type, static_registry<manager_type>>;

  using base_container_type    =
    basic_array_base_container<T, OptionalNoInitParam...>;

  using container_manager_type = container_manager_static<base_container_type>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the distribution type based on a
  /// container type.
  /// @tparam C Type of the container.
  //////////////////////////////////////////////////////////////////////
  template <typename C>
  struct construct_distribution
  {
    using type =
     distribution<
       C, operations::base, operations::iterable, operations::random_access>;
  };
}; // struct static_array_traits

} // namespace stapl

#endif // STAPL_CONTAINERS_STATIC_ARRAY_TRAITS_HPP

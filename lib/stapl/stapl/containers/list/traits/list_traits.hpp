/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_LIST_TRAITS_HPP
#define STAPL_CONTAINERS_LIST_TRAITS_HPP

#include "../list_gid.hpp"

#include <stapl/utility/directory.hpp>
#include <stapl/containers/distribution/directory/list_directory.hpp>

#include <stapl/containers/list/distribution.hpp>
#include <stapl/containers/list/traits/base_container_traits.hpp>
#include <stapl/containers/list/list_container_manager.hpp>
#include <stapl/containers/list/base_container.hpp>
#include <stapl/containers/list/null_registry.hpp>

#include <stapl/utility/use_default.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the @ref list container. Specifies
///        customizable type parameters that could be changed on a
///        per-container basis.
/// @ingroup plistTraits
///
/// @tparam T Type of the stored elements in the list.
/// @tparam PS Partition strategy that defines how to partition the
///            original domain into subdomains. The default partition
///            is @ref balanced_partition.
/// @tparam M Mapper that defines how to map the subdomains produced
///           by the partition to locations. The default mapper is
///           @ref mapper.
/// @see list.
//////////////////////////////////////////////////////////////////////
template <typename T, typename P, typename M>
struct list_traits
{
  typedef T                                          value_type;
  typedef P                                          partition_type;
  typedef M                                          mapper_type;

  typedef list_manager<P, M>                         manager_type;

  typedef list_base_container_traits<T>              base_container_traits_type;

  typedef list_base_container<
    base_container_traits_type>                      base_container_type;

  typedef typename base_container_type::gid_type     gid_type;

  typedef null_registry<gid_type>                    registry_type;

  typedef directory<
            gid_type, use_default,
            manager_type, registry_type
          >                                          base_directory_type;

  typedef list_directory<partition_type,
                         mapper_type,
                         base_directory_type>        directory_type;

  typedef list_container_manager<
    base_container_type>                             container_manager_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the distribution type based on a
  ///        container type.
  /// @tparam C Type of the container.
  //////////////////////////////////////////////////////////////////////
  template <typename C>
  struct construct_distribution
  {
    typedef list_distribution<C>   type;
  };
};

} // namespace stapl

#endif // STAPL_CONTAINERS_LIST_TRAITS_HPP

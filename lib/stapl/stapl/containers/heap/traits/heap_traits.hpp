/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_HEAP_TRAITS_HPP
#define STAPL_CONTAINERS_HEAP_TRAITS_HPP

#include <stapl/utility/directory.hpp>
#include <stapl/containers/distribution/directory/list_directory.hpp>
#include <stapl/containers/heap/distribution.hpp>

#include <stapl/containers/heap/traits/base_container_traits.hpp>
#include <stapl/containers/distribution/container_manager/cm_heap.hpp>
#include <stapl/containers/heap/base_container.hpp>

#include <stapl/containers/list/null_registry.hpp>

#include <stapl/utility/use_default.hpp>

#include "../heap_fwd.hpp"

namespace stapl{

//////////////////////////////////////////////////////////////////////
/// @brief Collection of types for @ref heap.
/// @ingroup pheapTraits
///
/// @tparam T Type of the stored elements in the container.
/// @tparam Comp The comparator used to inferred an ordering between
///   elements.
/// @tparam PS Partition strategy that defines how to partition
///   the original domain into subdomains.
/// @tparam Mapper Mapper that defines how to map the subdomains produced
///   by the partition to locations.
////////////////////////////////////////////////////////////////////////
template <typename T, typename Comp, typename PS, typename Mapper>
struct heap_traits
{
  typedef T                                         value_type;
  typedef Comp                                      comp_type;
  typedef PS                                        partition_type;
  typedef Mapper                                    mapper_type;

  typedef heap_base_container_traits<T,Comp>        base_container_traits_type;
  typedef heap_base_container
          <T,Comp,base_container_traits_type>       base_container_type;
  typedef typename base_container_type::gid_type    gid_type;
  typedef container_manager_heap
          <base_container_type>                     container_manager_type;
  typedef list_manager<PS, Mapper>                  manager_type;

  typedef directory<
    gid_type, use_default,
    manager_type, null_registry<gid_type>
  >                                                 base_directory_type;

  typedef list_directory<
    partition_type, mapper_type, base_directory_type
  >                                                 directory_type;

  template <typename C>
  struct construct_distribution
  {
    typedef heap_distribution<C> type;
  };
};

} // namespace stapl

#endif // STAPL_CONTAINERS_HEAP_TRAITS_HPP

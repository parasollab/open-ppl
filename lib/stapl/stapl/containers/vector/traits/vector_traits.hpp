/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_VECTOR_TRAITS_HPP
#define STAPL_CONTAINERS_VECTOR_TRAITS_HPP

#include <stapl/containers/distribution/directory/map_manager.hpp>
#include <stapl/containers/distribution/directory/interval_registry.hpp>
#include <stapl/containers/distribution/directory/vector_directory.hpp>

#include <stapl/containers/vector/distribution.hpp>
#include <stapl/containers/vector/vector_container_manager.hpp>
#include <stapl/containers/vector/base_container.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Collection of types for @ref vector.
/// @ingroup pvectorTraits
///
/// @tparam T Type of the stored elements in the container.
/// @tparam PS Partition strategy that defines how to partition
/// the original domain into subdomains.
/// @tparam Mapper Mapper that defines how to map the subdomains produced
/// by the partition to locations.
////////////////////////////////////////////////////////////////////////
template <typename T, typename PS, typename Mapper>
struct vector_traits
{
  typedef T                                                     value_type;
  typedef PS                                                    partition_type;
  typedef Mapper                                                mapper_type;
  typedef map_manager<PS,Mapper>                                manager_type;
  typedef interval_registry<manager_type>                       registry_type;

  typedef vector_directory<
    PS, Mapper, manager_type, registry_type
  >                                                             directory_type;

  typedef vector_base_container_traits<T>     base_container_traits_type;
  typedef vector_base_container
             <base_container_traits_type>     base_container_type;
  typedef typename base_container_type::gid_type         gid_type;

  typedef vector_container_manager
             <base_container_type>         container_manager_type;

  template <typename C>
  struct construct_distribution
  {
    typedef vector_distribution<C> type;
  };

  typedef indexed_domain<gid_type>                              domain_type;
};


namespace vector_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute traits class for @ref vector container.
///   For primary template, some optional parameters are defined, but no
///   customized traits.  Use @ref vector_traits.
//////////////////////////////////////////////////////////////////////
template<typename T, typename ...OptionalParams>
struct compute_vector_traits
{
private:
  typedef tuple<
    balanced_partition<indexed_domain<size_t>>, // Partition
    mapper<size_t>                              // Mapper
  >                                                       default_value_types;

  typedef typename compute_type_parameters<
    default_value_types, OptionalParams...
  >::type                                                 param_types;

  typedef typename tuple_element<0, param_types>::type    partition_t;
  typedef typename tuple_element<1, param_types>::type    mapper_t;

public:
  typedef vector_traits<T, partition_t, mapper_t>         type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when traits is actually passed at class
//    template instantiation. Use passed type.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Partition, typename Mapper, typename Traits>
struct compute_vector_traits<T, Partition, Mapper, Traits>
{
  typedef Traits type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when none of the optional parameters are
///   provided at template instantiation.  Use @ref vector_traits, with
///   default partition and mapper argument types.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct compute_vector_traits<T>
{
  typedef vector_traits<
    T, balanced_partition<indexed_domain<size_t>>, mapper<size_t>
  > type;
};

} // namespace vector_impl

} // namespace stapl

#endif // STAPL_CONTAINERS_VECTOR_TRAITS_HPP

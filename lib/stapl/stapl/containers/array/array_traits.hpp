/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_ARRAY_TRAITS_HPP
#define STAPL_CONTAINERS_ARRAY_TRAITS_HPP

#include <stapl/containers/distribution/distribution.hpp>
#include <stapl/containers/distribution/operations/base.hpp>
#include <stapl/containers/distribution/operations/iterable.hpp>
#include <stapl/containers/distribution/operations/random_access.hpp>

#include <stapl/containers/distribution/container_manager/container_manager.hpp>
#include <stapl/containers/mapping/mapper.hpp>

#include <stapl/domains/indexed.hpp>
#include <stapl/containers/array/base_container.hpp>
#include <stapl/containers/array/base_container_traits.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the array container. Specifies customizable
/// type parameters that could be changed on a per-container basis.
/// @ingroup parrayTraits
///
/// @tparam T Type of the stored elements in the array.
/// @tparam PS Partition strategy that defines how to partition
/// the original domain into subdomains. The default partition is
/// @ref balanced_partition.
/// @tparam M Mapper that defines how to map the subdomains produced
/// by the partition to locations. The default mapper is @ref mapper.
/// @see array
////////////////////////////////////////////////////////////////////////
template<typename T, typename P, typename M>
struct array_traits
{
  typedef T                                             value_type;
  typedef P                                             partition_type;
  typedef M                                             mapper_type;

  typedef indexed_domain<size_t>                        domain_type;
  typedef typename domain_type::index_type              index_type;
  typedef index_type                                    gid_type;
  typedef typename P::index_type                        cid_type;
  typedef typename domain_type::size_type               size_type;

  typedef basic_array_base_container<T>                 base_container_type;
  typedef container_manager<base_container_type>        container_manager_type;
  typedef container_directory<
    partition_type, mapper_type
  >                                                     directory_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the distribution type based on a
  /// container type.
  /// @tparam C Type of the container.
  //////////////////////////////////////////////////////////////////////
  template <typename C>
  struct construct_distribution
  {
    typedef distribution<
      C, operations::base, operations::iterable,
      operations::random_access, operations::migratable
    > type;
  };
}; // struct array_traits


namespace array_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute traits class for @ref array container.
///   For primary template, some optional parameters are defined, but no
///   customized traits.  Use @ref array_traits.
/// @ingroup parrayTraits
//////////////////////////////////////////////////////////////////////
template<typename T, typename ...OptionalParams>
struct compute_array_traits
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
  typedef array_traits<T, partition_t, mapper_t>           type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when traits is actually passed at class
//    template instantiation. Use passed type.
/// @ingroup parrayTraits
//////////////////////////////////////////////////////////////////////
template<typename T, typename Partition, typename Mapper, typename Traits>
struct compute_array_traits<T, Partition, Mapper, Traits>
{
  typedef Traits type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when none of the optional parameters are
///   provided at template instantiation.  Use @ref array_traits, with
///   default partition and mapper argument types.
/// @ingroup parrayTraits
//////////////////////////////////////////////////////////////////////
template<typename T>
struct compute_array_traits<T>
{
  typedef array_traits<
    T, balanced_partition<indexed_domain<size_t>>, mapper<size_t>
  > type;
};

} // namespace array_impl

} // namespace stapl

#endif // STAPL_CONTAINERS_ARRAY_TRAITS_HPP

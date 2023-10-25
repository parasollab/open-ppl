/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SET_SET_TRAITS_HPP
#define STAPL_CONTAINERS_SET_SET_TRAITS_HPP

#include <stapl/containers/distribution/directory/map_manager.hpp>
#include <stapl/containers/distribution/directory/interval_registry.hpp>
#include <stapl/containers/distribution/directory/vector_directory.hpp>
#include <stapl/containers/distribution/container_manager/associative_container_manager.hpp>

#include <stapl/containers/set/set_fwd.hpp>
#include <stapl/containers/set/distribution.hpp>
#include <stapl/containers/set/base_container.hpp>

#include <stapl/domains/iterator_domain.hpp>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/mapping/mapper.hpp>
#include <stapl/containers/set/base_container_traits.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the set container. Specifies customizable
/// type parameters that could be changed on a per-container basis.
/// @ingroup psetTraits
///
/// @tparam Key         Type of key objects.
/// @tparam Compare     Comparison function object type, defaults to
///                     @c std::less<Key>.
/// @tparam PS          Partition strategy that defines how to partition
///                     the original domain into subdomains. The default
///                     partition is @ref balanced_partition.
/// @tparam Mapper      Defines how to map the subdomains produced by
///                     the partition to locations. The default mapper
///                     is @ref mapper.
/// @tparam Traits      Defines customizable components of a parallel
///                     set, such as the domain type and base container
///                     type. The default traits class is @ref set_traits.
/// @see set
//////////////////////////////////////////////////////////////////////
template<typename Key,
         typename Compare = std::less<Key>,
         typename PS      = balanced_partition<indexed_domain<Key> >,
         typename Mapper  = mapper<size_t> >
struct set_traits
{
  typedef Key                                        key_type;
  typedef Key                                        value_type;
  typedef Compare                                    compare_type;
  typedef PS                                         partition_type;
  typedef Mapper                                     mapper_type;
  typedef map_manager<PS, Mapper>                    manager_type;
  typedef interval_registry<manager_type, Compare>   registry_type;

  typedef vector_directory<
    PS, Mapper, manager_type, registry_type
  >                                                  directory_type;

  typedef set_base_container_traits<Key,Compare>     base_container_traits_type;

  typedef set_base_container<
    Key,Compare, base_container_traits_type
  >                                                  base_container_type;

  typedef typename base_container_type::gid_type     gid_type;

  typedef associative_container_manager<
    base_container_type, manager_type
  >                                                  container_manager_type;

  using container_type = set<Key, Compare, PS, Mapper, set_traits>;
  using distribution_type = set_distribution<container_type>;
  using domain_type =
    iterator_domain<distribution_type, domain_impl::f_deref<gid_type>>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the distribution type based on a
  /// container type.
  /// @tparam C Type of the container.
  //////////////////////////////////////////////////////////////////////
  template <typename C>
  struct construct_distribution
  {
    typedef set_distribution<C> type;
  };
}; // struct set_traits


namespace set_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute traits class for @ref set container.
///   For primary template, some optional parameters are defined, but no
///   customized traits.  Use @ref set_traits.
/// @ingroup psetTraits
//////////////////////////////////////////////////////////////////////
template<typename Key, typename ...OptionalParams>
struct compute_set_traits
{
private:
  typedef tuple<
    std::less<Key>,                          // Compare
    balanced_partition<indexed_domain<Key>>, // Partition
    mapper<size_t>                           // Mapper
  >                                                         default_value_types;

  typedef typename compute_type_parameters<
    default_value_types, OptionalParams...
  >::type                                                   param_types;

  typedef typename tuple_element<0, param_types>::type      compare_t;
  typedef typename tuple_element<1, param_types>::type      partition_t;
  typedef typename tuple_element<2, param_types>::type      mapper_t;

public:
  typedef set_traits<Key, compare_t, partition_t, mapper_t> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when traits is actually passed at class
//    template instantiation. Use passed type.
/// @ingroup psetTraits
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Compare, typename Partition,
         typename Mapper, typename Traits>
struct compute_set_traits<Key, Compare, Partition, Mapper, Traits>
{
  typedef Traits type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when none of the optional parameters are
///   provided at template instantiation.  Use @ref set_traits, with
///   default partition and mapper argument types.
/// @ingroup psetTraits
//////////////////////////////////////////////////////////////////////
template<typename Key>
struct compute_set_traits<Key>
{
  typedef set_traits<
    Key, std::less<Key>,
    balanced_partition<indexed_domain<Key>>, mapper<size_t>
  > type;
};

} // namespace set_impl

} // namespace stapl

#endif // STAPL_CONTAINERS_SET_SET_TRAITS_HPP

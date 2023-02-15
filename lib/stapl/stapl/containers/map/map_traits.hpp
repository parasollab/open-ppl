/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_MAP_TRAITS_HPP
#define STAPL_MAP_TRAITS_HPP

#include <stapl/containers/distribution/directory/map_manager.hpp>
#include <stapl/containers/distribution/directory/interval_registry.hpp>

#include <stapl/containers/distribution/directory/vector_directory.hpp>

#include <stapl/containers/distribution/container_manager/associative_container_manager.hpp>
#include <stapl/containers/map/map_fwd.hpp>
#include <stapl/containers/map/distribution.hpp>
#include <stapl/containers/map/base_container.hpp>

#include <stapl/domains/iterator_domain.hpp>

#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/mapping/mapper.hpp>
#include <stapl/containers/map/map_base_container_traits.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the map container. Specifies customizable
/// type parameters that could be changed on a per-container basis.
/// @ingroup pmapTraits
///
/// @tparam Key Type of the key. Each element has a unique key value
/// @tparam T Type of the mapped value. Each element stores data of the
/// mapped type.
/// @tparam Comp Comparator used for less_than equality of keys.
/// @tparam PS Partitioner type. Used to specify how to partition the elements.
/// @tparam Mapper Mapper type. Maps the different partitions to the available
/// locations.
//////////////////////////////////////////////////////////////////////
template<typename Key, typename T,
typename Comp, typename PS, typename Mapper>
struct map_traits
{
  typedef Key                                      key_type;
  typedef T                                        mapped_type;
  typedef std::pair<const Key, T>                  value_type;
  typedef Comp                                     compare_type;
  typedef PS                                       partition_type;
  typedef Mapper                                   mapper_type;
  typedef map_manager<PS, Mapper>                  manager_type;

  typedef interval_registry<manager_type, Comp>    registry_type;

  typedef vector_directory<
    PS, Mapper, manager_type, registry_type
  >                                                directory_type;

  typedef map_base_container_traits<Key, T, Comp>  base_container_traits_type;

  typedef map_base_container<
    base_container_traits_type
  >                                                base_container_type;

  typedef typename base_container_type::gid_type   gid_type;

  typedef associative_container_manager<
    base_container_type, manager_type
  >                                                container_manager_type;

  using container_type = map<Key, T, Comp, PS, Mapper, map_traits>;
  using distribution_type = map_distribution<container_type>;
  using domain_type
    = iterator_domain<distribution_type, detail::get_first<Key>>;

  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the distribution type based on a
  /// container type.
  /// @tparam C Type of the container.
  //////////////////////////////////////////////////////////////////////
  template <typename C>
  struct construct_distribution
  {
    typedef map_distribution<C> type;
  };
}; // class map_traits


//////////////////////////////////////////////////////////////////////
/// @brief Traits type used when comparator, partition, and mapper are
///   the default values.
/// @ingroup pmapTraits
//////////////////////////////////////////////////////////////////////
template<typename Key, typename T>
struct basic_map_traits
  : public map_traits<
      Key, T,
      stapl::less<Key>,
      balanced_partition<indexed_domain<Key> >,
      mapper<size_t>
    >
{ };


namespace map_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute traits class for @ref map container.
///   For primary template, some optional parameters are defined, but no
///   customized traits.  Use @ref map_traits.
//////////////////////////////////////////////////////////////////////
template<typename Key, typename T, typename ...OptionalParams>
struct compute_map_traits
{
private:
  typedef tuple<
    stapl::less<Key>,                        // Comparator
    balanced_partition<indexed_domain<Key>>, // Partition
    mapper<size_t>                           // Mapper
  >                                                       default_value_types;

  typedef typename compute_type_parameters<
    default_value_types, OptionalParams...
  >::type                                                 param_types;

  typedef typename tuple_element<0, param_types>::type    comparator_t;
  typedef typename tuple_element<1, param_types>::type    partition_t;
  typedef typename tuple_element<2, param_types>::type    mapper_t;

public:
  typedef map_traits<
    Key, T, comparator_t, partition_t, mapper_t
  >                                                       type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when traits is actually passed at class
//    template instantiation. Use passed type.
//////////////////////////////////////////////////////////////////////
template<typename Key, typename T, typename Comp,
         typename Partition, typename Mapper, typename Traits>
struct compute_map_traits<Key, T, Comp, Partition, Mapper, Traits>
{
  typedef Traits type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when none of the optional parameters are
///   provided at template instantiation.  Use @ref basic_map_traits.
//////////////////////////////////////////////////////////////////////
template<typename Key, typename T>
struct compute_map_traits<Key, T>
{
  typedef basic_map_traits<Key, T> type;
};

} // namespace map_impl

} // namespace stapl

#endif // STAPL_MAP_TRAITS_HPP

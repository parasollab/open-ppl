/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MULTIMAP_TRAITS_HPP
#define STAPL_CONTAINERS_UNORDERED_MULTIMAP_TRAITS_HPP

#include <stapl/containers/distribution/directory/unordered_map_manager.hpp>
#include <stapl/containers/distribution/directory/unordered_registry.hpp>
#include <stapl/containers/distribution/directory/vector_directory.hpp>
#include <stapl/containers/distribution/container_manager/unordered_associative_container_manager.hpp>

#include "unordered_multimap_fwd.hpp"
#include <stapl/containers/unordered_multimap/distribution.hpp>
#include <stapl/containers/unordered_multimap/base_container.hpp>
#include <stapl/containers/unordered_multimap/base_container_traits.hpp>

#include <stapl/containers/unordered_map/unordered_map_partition.hpp>
#include <stapl/containers/mapping/mapper.hpp>
#include <stapl/domains/continuous.hpp>
#include <stapl/domains/iterator_domain.hpp>


namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the unordered_multimap container. Specifies
///   customizable type parameters that could be changed
///   on a per-container basis.
/// @ingroup punorderedmapTraits
/// @tparam Key Type of the key. Each element has a unique key value
/// @tparam Mapped Type of the mapped value. Each element stores data of the
///   mapped type.
/// @tparam Hash Unary Functor returning a unique value of type size_t. The
///   only argument is a type Key.
/// @tparam Pred Binary predicate used to compare key values of elements stored
///   in the container for equality.
/// @tparam PS Partionner type. Used to specify how to partition the elements.
/// @tparam M Mapper type. Maps the different partitions to the available
///   locations.
/// @see unordered_multimap
////////////////////////////////////////////////////////////////////////
template<typename Key, typename Map, typename Hash, typename Pred, typename PS,
  typename M>
struct unordered_multimap_traits
{
  typedef unordered_multimap<
            Key, Map, Hash, Pred, PS, M, unordered_multimap_traits
          >                                          container_type;

  typedef unordered_multimap_base_container_traits<
    Key, Map, Hash, Pred>                            base_container_traits_type;
  typedef unordered_multimap_base_container<
    base_container_traits_type>                      base_container_type;

  typedef Key                                        key_type;
  typedef Map                                        mapped_type;
  typedef typename base_container_type::gid_type     gid_type;
  typedef std::pair<const gid_type, mapped_type>     value_type;
  typedef value_type                                 stored_type;
  typedef Hash                                       hasher;
  typedef Pred                                       key_equal;
  typedef PS                                         partition_type;
  typedef M                                          mapper_type;

  typedef unordered_map_manager<
    partition_type, mapper_type>                     manager_type;
  typedef vector_directory<
    partition_type, mapper_type, manager_type,
    unordered_registry<manager_type> >               directory_type;
  typedef unordered_associative_container_manager<
    base_container_type, manager_type>               container_manager_type;
  typedef iterator_domain<
    unordered_multimap_distribution<container_type>,
    domain_impl::f_deref_gid<gid_type> >             domain_type;


  //////////////////////////////////////////////////////////////////////
  /// @brief Metafunction to compute the distribution type based on a
  /// container type.
  /// @tparam C Type of the container.
  //////////////////////////////////////////////////////////////////////
  template <typename C>
  struct construct_distribution
  {
    typedef unordered_multimap_distribution<C>  type;
    typedef typename type::key_type             key_type;
    typedef typename type::mapped_type          mapped_type;
    typedef typename type::stored_type          stored_type;
  };
}; // struct unordered_multimap_traits

namespace unordered_multimap_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute traits class for the @ref unordered
///   multimap container. For primary template, some optional parameters are
///   defined, but no customized traits. Use @ref unordered_multimap_traits.
/// @ingroup punorderedmultimapTraits
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Map, typename ...OptionalParams>
struct compute_unordered_multimap_traits
{
private:
  typedef tuple<
    stapl::hash<Key>,                       // Hasher
    stapl::equal_to<multi_key<Key> >,       // Predicate
    unordered_map_partition<
      pair_hash<Key, stapl::hash<Key> >,
      continuous_domain<multi_key<Key> > >, // Partition
    mapper<size_t>                          // Mapper
  >                                                       default_value_types;

  typedef typename compute_type_parameters<
    default_value_types, OptionalParams...
  >::type                                                 param_types;

  typedef typename tuple_element<0, param_types>::type    hash_t;
  typedef typename tuple_element<1, param_types>::type    pred_t;
  typedef typename tuple_element<2, param_types>::type    partition_t;
  typedef typename tuple_element<3, param_types>::type    mapper_t;

public:
  typedef
    unordered_multimap_traits<Key,Map,hash_t,pred_t,partition_t,mapper_t> type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization when traits is actually passed at class
//    template instantiation. Use passed type.
/// @ingroup punorderedmultimapTraits
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Map, typename Hash, typename Pred, typename PS,
  typename M, typename Traits>
struct compute_unordered_multimap_traits<Key, Map, Hash, Pred, PS, M, Traits>
{
  typedef Traits type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization when none of the optional parameters are
///   provided at template instantiation.  Use @ref unordered_multimap_traits,
///   with default partition and mapper argument types.
/// @ingroup punorderedmultimapTraits
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Map>
struct compute_unordered_multimap_traits<Key, Map>
{
  typedef unordered_multimap_traits<
    Key, Map, stapl::hash<Key>, stapl::equal_to<multi_key<Key> >,
    unordered_map_partition<
      pair_hash<Key, stapl::hash<Key> >,
      continuous_domain<multi_key<Key> > >,
    mapper<size_t>
  > type;
};

} // namespace unordered_multimap_impl

} // namespace stapl

#endif

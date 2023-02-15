/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SET_SET_FWD_HPP
#define STAPL_CONTAINERS_SET_SET_FWD_HPP

#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/mapping/mapper.hpp>
#include <stapl/domains/indexed.hpp>

namespace stapl {

#ifdef STAPL_DOCUMENTATION_ONLY

//////////////////////////////////////////////////////////////////////
/// @brief A parallel counterpart of the sequential set. It provides
/// the functionality for inserting, erasing, retrieving elements in
/// a parallel @c set.
/// @ingroup pset
///
/// @tparam Key         Type of key objects.
/// @tparam Compare     Comparison function object type, defaults to
///                     @c std::less<Key>.
/// @tparam Partitioner Partition strategy that defines how to partition
///                     the original domain into subdomains. The default
///                     partition is @ref balanced_partition.
/// @tparam Mapper      Defines how to map the subdomains produced by
///                     the partition to locations. The default mapper
///                     is @ref mapper.
/// @tparam Traits      Defines customizable components of a parallel
///                     set, such as the domain type and base container
///                     type. The default traits class is @ref set_traits.
//////////////////////////////////////////////////////////////////////
template<typename Key,
         typename Compare     = std::less<Key>,
         typename Partitioner = balanced_partition<indexed_domain<Key> >,
         typename Mapper      = mapper<size_t>,
         typename Traits      = set_traits<Key, Compare, Partitioner, Mapper> >
class set;

#else

template<typename Key, typename ...OptionalParams>
class set;

#endif // STAPL_DOCUMENTATION_ONLY

} // namespace stapl

#endif // STAPL_CONTAINERS_SET_SET_FWD_HPP

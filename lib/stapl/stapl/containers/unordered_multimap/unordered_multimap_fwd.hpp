/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_UNORDERED_MULTIMAP_FWD_HPP
#define STAPL_CONTAINERS_UNORDERED_MULTIMAP_FWD_HPP

namespace stapl {

#ifdef STAPL_DOCUMENTATION_ONLY

//////////////////////////////////////////////////////////////////////
/// @brief Parallel hash-based unordered multimap container.
/// @ingroup punorderedmultimap
/// @tparam Key Type of the key.
/// @tparam Mapped Type of the mapped values.
/// @tparam Hash Unary functor returning a unique value of type size_t. The
///   only argument is of type Key. The default hash is stapl::hash<Key>.
/// @tparam Pred Binary predicate used to compare key values of elements stored
///   in the container for equality. The default predicate
///   is stapl::equal_to<multi_key<Key> >.
/// @tparam PS Partition type. Specifies how to partition elements. The default
///   is unordered_map_partition<pair_hash<Key, Hash>,
///   continuous_domain<multi_key<Key> > >.
/// @tparam M Mapper type. Maps the different partitions to the available
///   locations. The default mapper is @ref mapper.
/// @tparam Traits Traits type. Specifies the internal configuration of the
///   container. The default traits is unordered_set_traits.
//////////////////////////////////////////////////////////////////////
template<typename Key,
         typename Mapped,
         typename Hash  = use_default,
         typename Pred  = use_default,
         typename PS    = use_default,
         typename M     = use_default,
         typename Traits= use_default >
class unordered_multimap;

#else

template<typename Key, typename Mapped, typename ...OptionalParams>
class unordered_multimap;

#endif // STAPL_DOCUMENTATION_ONLY

} // namespace stapl

#endif // STAPL_CONTAINERS_UNORDERED_MULTIMAP_FWD_HPP

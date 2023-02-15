/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MAP_FWD_HPP
#define STAPL_CONTAINERS_MAP_FWD_HPP

#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/mapping/mapper.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/utility/use_default.hpp>
#include <stapl/algorithms/functional.hpp>

namespace stapl {

#ifdef STAPL_DOCUMENTATION_ONLY

//////////////////////////////////////////////////////////////////////
/// @brief Parallel ordered map container. Insertion and deletion does
///   invalidate iterators pointing to existing elements.
/// @ingroup pmap
/// @tparam Key Type of the key. Each element has a unique key value
/// @tparam Mapped Type of the mapped value. Each element stores data of the
///   mapped type.
/// @tparam Comp Comparator used for less_than equality of keys. Default
///   comparator is @ref stapl::less.
/// @tparam PS Partitioner type. Used to specify how to partition the elements.
///   The default partition is @ref balanced_partition of an
///   @ref indexed_domain.
/// @tparam M Mapper type. Maps the different partitions to the available
///   locations. The default mapper is @ref mapper.
/// @tparam Traits Traits type. Specifies the internal configuration of the
///   container. The default traits is @ref map_traits.
/// @todo Comparator should be passed in and stored in this class.
//////////////////////////////////////////////////////////////////////
template<typename Key,
         typename T,
         typename Comp        = stapl::less<Key>,
         typename Partitioner = use_default,
         typename Mapper      = use_default,
         typename Traits      = use_default>
class map;

#else

template<typename Key, typename T, typename ...OptionalParams>
class map;

#endif // STAPL_DOCUMENTATION_ONLY

} // namespace stapl

#endif // STAPL_CONTAINERS_MAP_FWD_HPP

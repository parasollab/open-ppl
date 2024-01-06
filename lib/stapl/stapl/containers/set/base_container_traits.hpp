/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_SET_BASE_CONTAINER_TRAITS_HPP
#define STAPL_CONTAINERS_SET_BASE_CONTAINER_TRAITS_HPP

#include <set>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for std::set. Used
///   for @ref set_base_container.
/// @ingroup psetTraits
/// @see set_base_container container_traits
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Compare>
struct container_traits<std::set<Key, Compare>>
{
  typedef Key                                  gid_type;
  typedef Key                                  index_type;
  typedef Key                                  value_type;
  typedef std::set<Key,Compare>                container_type;
  typedef iterator_domain<container_type>      domain_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the set base container. Specifies customizable
/// type parameters that could be changed on a per-container basis.
/// @ingroup psetTraits
///
/// @tparam Key Type of the key. Each element has a unique key value
/// @tparam Compare Comparator used for less_than equality of keys.
//////////////////////////////////////////////////////////////////////
template<typename Key, typename Compare>
struct set_base_container_traits
{
  typedef Key                                   key_type;
  typedef Key                                   value_type;
  typedef Compare                               compare_type;
  typedef std::set<Key,Compare>                 container_type;
  typedef iterator_domain<container_type>       domain_type;
};

} // namespace stapl

#endif // STAPL_CONTAINERS_SET_BASE_CONTAINER_TRAITS_HPP

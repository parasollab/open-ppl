/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MAP_BASE_CONTAINER_TRAITS_HPP
#define STAPL_CONTAINERS_MAP_BASE_CONTAINER_TRAITS_HPP

#include <stapl/containers/type_traits/define_value_type.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the map base container. Specifies customizable
/// type parameters that could be changed on a per-container basis.
/// @ingroup pmapTraits
///
/// @tparam Key Type of the key. Each element has a unique key value
/// @tparam T Type of the mapped value. Each element stores data of the
/// mapped type.
/// @tparam Compare Comparator used for less_than equality of keys.
//////////////////////////////////////////////////////////////////////
template<typename Key, typename T, typename Compare>
struct map_base_container_traits
{
  typedef typename define_value_type<T>::type      stored_type;
  typedef Key                                      gid_type;
  typedef Key                                      key_type;
  typedef T                                        mapped_type;
  typedef std::pair<const Key, T>                  value_type;
  typedef Compare                                  compare_type;
  typedef std::map<Key, stored_type, Compare>      container_type;
  typedef iterator_domain<
    container_type, detail::get_first<Key>
  >                                                domain_type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of @ref container_traits for std::map. Used
/// for @ref map_base_container.
/// @ingroup pmapTraits
/// @see map_base_container container_traits
//////////////////////////////////////////////////////////////////////
template<typename Key, typename T, typename Compare>
struct container_traits<std::map<Key, T, Compare> >
{
private:
  typedef typename define_value_type<T>::type          stored_type;

public:
  typedef Key                                          gid_type;
  typedef Key                                          index_type;
  typedef std::pair<const Key, T>                      value_type;
  typedef typename std::map<
    Key, stored_type, Compare
  >::iterator                                          iterator;
};

} // namespace stapl

#endif

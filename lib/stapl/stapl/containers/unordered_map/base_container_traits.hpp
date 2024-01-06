/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_UNORDERED_MAP_BASE_CONTAINER_TRAITS_HPP
#define STAPL_CONTAINERS_UNORDERED_MAP_BASE_CONTAINER_TRAITS_HPP

#include <boost/unordered_map.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/domains/iterator_domain.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>
#include <stapl/containers/type_traits/define_value_type.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the unordered_map_base_container. Specifies
///   customizable type parameters that could be changed on a
///   per-container basis.
/// @ingroup punorderedmapTraits
/// @tparam Key Type of the key. Each element has a unique key value
/// @tparam Mapped Type of the mapped value. Each element stores data of the
///   mapped type.
/// @tparam Hash Unary Functor returning a unique value of type size_t. The
///   only argument is a type Key.
/// @tparam Pred Binary predicate used to compare key values of elements stored
///   in the container for equality.
/// @see unordered_map_base_container
////////////////////////////////////////////////////////////////////////
template<typename Key, typename Mapped, typename Hash, typename Pred>
struct unordered_map_base_container_traits
{
private:
  typedef typename define_value_type<Mapped>::type       elem_type;

public:
  typedef Key                                            gid_type;
  typedef Key                                            key_type;
  typedef Mapped                                         mapped_type;
  typedef std::pair<const Key, Mapped>                   value_type;
  typedef value_type                                     stored_type;
  typedef Pred                                           compare_type;
  typedef std::unordered_map<Key, elem_type, Hash, Pred> container_type;
  typedef iterator_domain<
    container_type, detail::get_first<Key>
  >                                                      domain_type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of container_traits for the boost::unordered_map.
/// @ingroup punorderedmapTraits
/// @tparam Key Type of the key. Each element has a unique key value
/// @tparam T Type of the mapped value. Each element stores data of the
///   mapped type.
/// @tparam H Unary Functor returning a unique value of type size_t. The
///   only argument is a type Key.
/// @tparam P Binary predicate used to compare key values of elements stored
///   in the container for equality.
////////////////////////////////////////////////////////////////////////
template<typename Key, typename T, typename H, typename P>
struct container_traits<std::unordered_map<Key, T, H, P> >
{
private:
  typedef typename define_value_type<T>::type      elem_type;

public:
  typedef Key                                      gid_type;
  typedef Key                                      index_type;
  typedef std::pair<const Key,T>                   value_type;
  typedef value_type                               stored_type;
  typedef typename std::unordered_map<
    Key, elem_type, H, P>::iterator                iterator;
};

} // namespace stapl

#endif

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_UNORDERED_MULTIMAP_BASE_CONTAINER_TRAITS_HPP
#define STAPL_CONTAINERS_UNORDERED_MULTIMAP_BASE_CONTAINER_TRAITS_HPP

#include <boost/unordered_map.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/domains/iterator_domain.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the unordered_multimap_base_container. Specifies
///   customizable type parameters that could be changed on a
///   per-container basis.
/// @ingroup punorderedmapTraits
/// @tparam Key Type of the key.
/// @tparam Mapped Type of the mapped value. Each element stores data of the
///   mapped type.
/// @tparam Hash Unary Functor returning a unique value of type size_t. The
///   only argument is a type Key.
/// @tparam Pred Binary predicate used to compare key values of elements stored
///   in the container for equality.
/// @see unordered_multimap_base_container
////////////////////////////////////////////////////////////////////////
template<typename Key, typename Mapped, typename Hash, typename Pred>
struct unordered_multimap_base_container_traits
{
  typedef Key                                                key_type;
  typedef Mapped                                             mapped_type;
  typedef multi_key<key_type>                                gid_type;
  typedef std::pair<const gid_type, mapped_type>             value_type;
  typedef value_type                                         stored_type;
  typedef Pred                                               compare_type;
  typedef boost::unordered_multimap<gid_type,
            Mapped, Hash, Pred>                              container_type;
  typedef iterator_domain<
    container_type, domain_impl::f_deref_gid<gid_type>
  >                                                          domain_type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of container_traits for the boost::unordered_multimap.
/// @ingroup punorderedmultimapTraits
/// @tparam Key Type of the key.
/// @tparam T Type of the mapped value. Each element stores data of the
///   mapped type.
/// @tparam H Unary Functor returning a unique value of type size_t. The
///   only argument is a type Key.
/// @tparam P Binary predicate used to compare key values of elements stored
///   in the container for equality.
////////////////////////////////////////////////////////////////////////
template<typename Key, typename T, typename H, typename P>
struct container_traits<boost::unordered_multimap<multi_key<Key>, T, H, P> >
{
  typedef multi_key<Key>                gid_type;
  typedef gid_type                      index_type;
  typedef std::pair<const gid_type, T>  value_type;
  typedef value_type                    stored_type;
};

} // namespace stapl

#endif

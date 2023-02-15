/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_UNORDERED_MULTISET_BASE_CONTAINER_TRAITS_HPP
#define STAPL_CONTAINERS_UNORDERED_MULTISET_BASE_CONTAINER_TRAITS_HPP

#include <boost/unordered_set.hpp>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/domains/iterator_domain.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the unordered_multiset_base_container.
///   Specifies customizable type parameters that could be changed on a
///   per-container basis.
/// @ingroup punorderedmultisetTraits
/// @tparam Key Type of the key.
/// @tparam Hash Unary functor returning a unique value of type size_t. The
///   only argument is of type Key.
/// @tparam Pred Binary predicate used to compare key values of elements stored
///   in the container for equality.
/// @see unordered_multiset_base_container
////////////////////////////////////////////////////////////////////////
template<typename Key, typename Hash, typename Pred>
struct unordered_multiset_base_container_traits
{
  typedef Key                                                key_type;
  typedef Key                                                mapped_type;
  typedef Key                                                value_type;
  typedef multi_key<Key>                                     stored_type;
  typedef stored_type                                        gid_type;
  typedef boost::unordered_multiset<stored_type, Hash, Pred> container_type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of container_traits for the boost::unordered_multiset.
/// @ingroup punorderedmultisetTraits
/// @tparam Key Type of the key.
/// @tparam H Unary functor returning a unique value of type size_t. The
///   only argument is of type Key.
/// @tparam P Binary predicate used to compare key values of elements stored
///   in the container for equality.
////////////////////////////////////////////////////////////////////////
template<typename Key, typename H, typename P>
struct container_traits<boost::unordered_multiset<multi_key<Key>, H, P> >
{
  typedef multi_key<Key> gid_type;
  typedef Key            key_type;
  typedef Key            mapped_type;
  typedef Key            value_type;
  typedef gid_type       stored_type;
  typedef gid_type       index_type;

};

} // namespace stapl

#endif

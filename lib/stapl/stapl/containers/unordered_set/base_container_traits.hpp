/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_UNORDERED_SET_BASE_CONTAINER_TRAITS_HPP
#define STAPL_CONTAINERS_UNORDERED_SET_BASE_CONTAINER_TRAITS_HPP

#include <unordered_set>
#include <stapl/containers/type_traits/container_traits.hpp>
#include <stapl/domains/iterator_domain.hpp>
#include <stapl/containers/iterators/local_iterator.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the unordered_set_base_container. Specifies
///   customizable type parameters that could be changed on a
///   per-container basis.
/// @ingroup punorderedsetTraits
/// @tparam Key Type of the key. Each element has a unique key value
/// @tparam Hash Unary Functor returning a unique value of type size_t. The
///   only argument is a type Key.
/// @tparam Pred Binary predicate used to compare key values of elements stored
///   in the container for equality.
/// @see unordered_set_base_container
////////////////////////////////////////////////////////////////////////
template<typename Key, typename Hash, typename Pred>
struct unordered_set_base_container_traits
{
  typedef Key                                   key_type;
  typedef Key                                   mapped_type;
  typedef Key                                   value_type;
  typedef std::unordered_set<Key, Hash, Pred>   container_type;
  typedef iterator_domain<container_type>       domain_type;
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization of container_traits for the boost::unordered_set.
/// @ingroup punorderedsetTraits
/// @tparam Key Type of the key. Each element has a unique key value
/// @tparam H Unary Functor returning a unique value of type size_t. The
///   only argument is a type Key.
/// @tparam P Binary predicate used to compare key values of elements stored
///   in the container for equality.
////////////////////////////////////////////////////////////////////////
template<typename Key, typename H, typename P>
struct container_traits<std::unordered_set<Key, H, P> >
{
  typedef Key                             gid_type;
  typedef Key                             mapped_type;
  typedef Key                             index_type;
  typedef Key                             value_type;
  typedef std::unordered_set<Key, H, P>   container_type;
  typedef iterator_domain<container_type> domain_type;
};

} // namespace stapl

#endif

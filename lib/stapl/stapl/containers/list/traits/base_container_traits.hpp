/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_LIST_TRAITS_LIST_BASE_CONTAINER_TRAITS_HPP
#define STAPL_CONTAINERS_LIST_TRAITS_LIST_BASE_CONTAINER_TRAITS_HPP

#include <list>

#include <stapl/containers/type_traits/define_value_type.hpp>
#include <stapl/domains/iterator_domain.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the list base container. Specifies
///        customizable type parameters that could be changed on a
///        per-container basis.
///
/// @ingroup plistTraits
/// @tparam T Type of the stored elements in the base container.
/// @see list_base_container
////////////////////////////////////////////////////////////////////////
template<typename T>
struct list_base_container_traits
{
  typedef typename define_value_type<T>::type     stored_type;

  typedef std::list<stored_type>                  container_type;
  typedef T                                       value_type;
};

} // stapl namespace

#endif /* STAPL_CONTAINERS_LIST_TRAITS_LIST_BASE_CONTAINER_TRAITS_HPP */

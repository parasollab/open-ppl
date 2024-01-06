/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_VECTOR_BASE_CONTAINER_TRAITS_HPP
#define STAPL_CONTAINERS_VECTOR_BASE_CONTAINER_TRAITS_HPP

#include <vector>

#include <stapl/domains/indexed.hpp>
#include <stapl/containers/type_traits/define_value_type.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the vector base container. These traits
///        can be changed to customize base containers.
/// @ingroup pvectorTraits
/// @tparam T Type of the stored elements in the base container.
/// @ingroup pvectorTraits
////////////////////////////////////////////////////////////////////////
template <typename T>
struct vector_base_container_traits
{
  typedef typename define_value_type<T>::type   stored_type;
  typedef std::vector<stored_type>              container_type;
  typedef container_type                        container_constructor;
  typedef T                                     value_type;
  typedef indexed_domain<size_t>                domain_type;
};

} // namespace stapl

#endif // STAPL_CONTAINERS_VECTOR_BASE_CONTAINER_TRAITS_HPP

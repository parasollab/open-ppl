/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_MULTIARRAY_BASE_CONTAINER_TRAITS_H
#define STAPL_CONTAINERS_MULTIARRAY_BASE_CONTAINER_TRAITS_H

#include <vector>
#include <boost/mpl/int.hpp>
#include <stapl/containers/type_traits/define_value_type.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Default traits for the multiarray base container. Specifies
///   customizable type parameters that could be changed on a
///   per-container basis.
/// @ingroup pmultiarrayTraits
/// @tparam T Type of the stored elements in the base container.
/// @tparam Domain Domain for the base container.
/// @see multiarray_base_container
////////////////////////////////////////////////////////////////////////
template <typename T, int Dim, typename Traversal>
struct multiarray_base_container_traits
{
  typedef typename define_value_type<T>::type            stored_type;
  typedef std::vector<stored_type>                       container_type;
  typedef std::integral_constant<int, Dim>               dimension_type;
  typedef Traversal                                      traversal_type;
  typedef container_type                                 container_constructor;
  typedef T                                              value_type;
};

} // namespace stapl

#endif

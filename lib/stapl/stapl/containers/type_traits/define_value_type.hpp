/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DEFINE_VALUE_TYPE_HPP
#define STAPL_CONTAINERS_DEFINE_VALUE_TYPE_HPP

#include <stapl/containers/type_traits/is_container.hpp>
#include <stapl/containers/type_traits/container_wrapper_ref.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to compute the stored value of an element in
/// a container. In the cases of nested containers, it is necessary
/// to store a reference to the container, as it is a p_object.
///
/// @tparam T The container's value_type
/// @tparam Tag Whether T is a parallel container
//////////////////////////////////////////////////////////////////////
template <typename T, bool = is_container<T>::value>
struct define_value_type
{
  typedef T type;
};


template <typename T>
struct define_value_type<T, true>
{
private:
  typedef T                                 value_type;

public:
  typedef container_wrapper_ref<value_type> type;
};

} // namespace stapl

#endif // STAPL_CONTAINERS_DEFINE_VALUE_TYPE_HPP

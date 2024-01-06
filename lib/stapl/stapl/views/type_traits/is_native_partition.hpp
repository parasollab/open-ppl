/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_TYPE_TRAITS_IS_NATIVE_PARTITION_HPP
#define STAPL_CONTAINERS_TYPE_TRAITS_IS_NATIVE_PARTITION_HPP

#include <stapl/views/native_view_fwd.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Type checker to determine if a partition is a
///        native_partition type.
//////////////////////////////////////////////////////////////////////
template<typename P>
struct is_native_partition
  : boost::false_type
{ };


template<typename V>
struct is_native_partition<view_impl::native_partition<V> >
  : boost::true_type
{ };

} // stapl namespace


#endif /* STAPL_CONTAINERS_TYPE_TRAITS_IS_NATIVE_PARTITION_HPP */

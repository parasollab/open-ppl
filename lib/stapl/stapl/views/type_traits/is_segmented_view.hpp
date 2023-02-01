/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TYPE_TRAITS_IS_SEGMENTED_VIEW_HPP
#define STAPL_VIEWS_TYPE_TRAITS_IS_SEGMENTED_VIEW_HPP

#include <stapl/views/segmented_view_base.hpp>
#include <stapl/views/view_traits.hpp>
#include <type_traits>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Type checker to determine if the given @p View type is an
///        instantiation of segmented_view.
//////////////////////////////////////////////////////////////////////
template <typename View, bool = is_view<View>::value>
struct is_segmented_view
  : std::integral_constant<bool,
      std::is_base_of<segmented_view_base, View>::value ||
      std::is_base_of<view_container_base,
        typename view_traits<View>::container
      >::value
    >
{ };

template <typename View>
struct is_segmented_view<View, false>
  : std::false_type
{ };

} //namespace stapl

#endif // STAPL_VIEWS_TYPE_TRAITS_IS_SEGMENTED_VIEW_HPP

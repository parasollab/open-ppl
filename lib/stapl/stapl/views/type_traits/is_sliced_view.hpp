/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TYPE_TRAITS_IS_SLICED_VIEW_HPP
#define STAPL_VIEWS_TYPE_TRAITS_IS_SLICED_VIEW_HPP

#include <stapl/views/segmented_view_base.hpp>
#include <stapl/views/view_traits.hpp>
#include <type_traits>

namespace stapl {

template<typename Slices, typename Container, typename... OptParams>
class SLICED_view;

//////////////////////////////////////////////////////////////////////
/// @brief Type checker to determine if the given @p View type is an
///        instantiation of @ref SLICED_view.
//////////////////////////////////////////////////////////////////////
template<typename View>
struct is_SLICED_view
  : std::false_type
{ };

template<typename Slices, typename Container, typename... OptParams>
struct is_SLICED_view<SLICED_view<Slices, Container, OptParams...>>
  : std::true_type
{ };

} //namespace stapl

#endif // STAPL_VIEWS_TYPE_TRAITS_IS_SLICED_VIEW_HPP

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TYPE_TRAITS_IS_VIEW_HPP
#define STAPL_VIEWS_TYPE_TRAITS_IS_VIEW_HPP

#include <type_traits>

namespace stapl {

struct base_view;

//////////////////////////////////////////////////////////////////////
/// @brief Type checker to determine if a variable is an instantiation
///        of view.
//////////////////////////////////////////////////////////////////////
template <typename View>
struct is_view
  : std::is_base_of<base_view, View>
{ };

} // namespace stapl

#endif // STAPL_VIEWS_TYPE_TRAITS_IS_VIEW_HPP

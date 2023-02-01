/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_STRIP_FAST_VIEW_HPP
#define STAPL_VIEWS_STRIP_FAST_VIEW_HPP

namespace stapl {
namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Computes the type of the view underlying an @ref localized_view
/// instance.
///
/// Used in the implementation of @ref unique_copy to create a view over
/// elements to be copied using the array_view::set_elements method
//////////////////////////////////////////////////////////////////////
template<typename T>
struct strip_fast_view
{
  using type = T;
};

template<typename View>
struct strip_fast_view<localized_view<View>>
{
  using type = typename cast_container_view<
                 typename View::base_type, typename View::component_type
               >::type;
};


//////////////////////////////////////////////////////////////////////
/// @brief Computes the type of the view underlying a @ref mix_view
/// instance.
///
/// Used in the implementation of @ref unique_copy to create a view over
/// elements to be copied using the array_view::set_elements method
//////////////////////////////////////////////////////////////////////
template<typename T>
struct strip_mix_view
{
  using type = T;
};

template<typename View, typename Info, typename CID>
struct strip_mix_view<mix_view<View, Info, CID>>
{
  using type = View;
};

//////////////////////////////////////////////////////////////////////
/// @brief Computes the type of the view underlying an @ref repeat_view
/// instance.
///
/// Used in the implementation of @ref unique_copy to create a view over
/// elements to be copied using the array_view::set_elements method
//////////////////////////////////////////////////////////////////////
template<typename T>
struct strip_repeat_view
{
  using type = T;
};

template<typename T, int N, bool IsReadOnly, bool IsView>
struct strip_repeat_view<
         repeat_view<view_impl::repeat_container<T, N, IsReadOnly, IsView>>>
{
  using type = T;
};

} // namespace detail
} // namespace stapl

#endif // STAPL_VIEWS_STRIP_FAST_VIEW_HPP

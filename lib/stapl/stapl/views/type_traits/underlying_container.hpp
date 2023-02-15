/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_TYPE_TRAITS_UNDERLYING_CONTAINER_HPP
#define STAPL_VIEWS_TYPE_TRAITS_UNDERLYING_CONTAINER_HPP

#include <stapl/views/type_traits/is_segmented_view.hpp>
#include <stapl/views/type_traits/is_view.hpp>
#include <type_traits>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to extract the bottom-most container from a view
///        type.
///
/// For composed views, this goes through each view type and returns its
/// container until the container is not a view. For segmented_views
/// this returns the container of the view that is being segmented.
//////////////////////////////////////////////////////////////////////
template<
  typename T,
  bool ViewContainerIsView = is_view<
    typename T::view_container_type
  >::type::value,
  bool IsSegmented = is_segmented_view<T>::type::value
>
struct underlying_container_impl
{
  using recursive = underlying_container_impl<typename T::view_container_type>;
  using type = typename recursive::type;

  static type const& get(T const& view)
  {
    return recursive::get(view.container());
  }

  static type& get_mutable(T& view)
  {
    return recursive::get_mutable(view.container());
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for composed segmented_views
//////////////////////////////////////////////////////////////////////
template<typename T>
struct underlying_container_impl<T, true, true>
{
  using recursive = underlying_container_impl<
    typename T::view_container_type::view_container_type
  >;

  using type = typename recursive::type;

  static type const& get(T const& view)
  {
    return recursive::get(view.container().container());
  }

  static type& get_mutable(T& view)
  {
    return recursive::get_mutable(view.container().container());
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Specialization when we've reached the bottom.
//////////////////////////////////////////////////////////////////////
template<typename T>
struct underlying_container_impl<T, false, false>
{
  using type = typename T::view_container_type;

  static type const& get(T const& view)
  {
    return view.container();
  }

  static type& get_mutable(T& view)
  {
    return view.container();
  }
};

} // namespace detail

template<typename T, bool IsView = is_view<T>::type::value>
struct underlying_container
 : public detail::underlying_container_impl<T>
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for non-views
//////////////////////////////////////////////////////////////////////
template<typename T>
struct underlying_container<T, false>
{
  using type = T;

  static T const& get(T const& view)
  {
    return view;
  }

  static T& get_mutable(T& view)
  {
    return view;
  }
};

template<typename View>
using underlying_container_t = typename underlying_container<View>::type;

} //namespace stapl

#endif // STAPL_VIEWS_TYPE_TRAITS_UNDERLYING_CONTAINER_HPP

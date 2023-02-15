/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_IS_NESTED_CONSTRUCTION_HPP
#define STAPL_VIEWS_IS_NESTED_CONSTRUCTION_HPP

#include <stapl/runtime/type_traits/is_p_object.hpp>
#include <stapl/views/type_traits/is_view.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Type checker to determine if @p T is a @ref p_object, query
///        it about whether nested_construction occurred.  Otherwise,
///        return false.
//////////////////////////////////////////////////////////////////////
template <typename T, bool = is_p_object<T>::value>
struct nc_detect_pobject
{
  static bool call(T const&)
  {
    return false;
  }
};


template <typename T>
struct nc_detect_pobject<T, true>
{
  static bool call(T const& t)
  {
    return t.nested_construction();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Type checker to determine if @p T is a nested construct or
///        not.
///
/// @todo nc_detect_type really has nothing to do with nesting, it's a
///       general idiom of peeling back view / proxies and applying a
///       functor (in this case, nc_detect_type).  Look around and see
///       if this already exists in the codebase or generalize and
///       place elsewhere.
//////////////////////////////////////////////////////////////////////
template <typename T, bool = is_view<T>::value>
struct nc_detect_type
{
  typedef bool result_type;

  bool operator()(T const& t) const
  {
    return nc_detect_pobject<T>::call(t);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for proxies.
//////////////////////////////////////////////////////////////////////
template <typename T, typename A, bool b_is_view>
struct nc_detect_type<proxy<T, A>, b_is_view>
{
  typedef bool result_type;

  bool operator()(proxy<T,A> const& p) const
  {
    return accessor_core_access::apply_get(
      proxy_core_access::accessor(p),
      nc_detect_type<T>()
    );
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for views.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct nc_detect_type<T, true> // T is a view
{
  typedef bool result_type;

  bool operator()(T const& t) const
  {
    typedef typename T::view_container_type view_container_t;

    return nc_detect_type<view_container_t>()(t.container());
  }
};

} // namespace detail


//////////////////////////////////////////////////////////////////////
/// @brief Function to determine whether @p t is a nested construct.
//////////////////////////////////////////////////////////////////////
template <typename T>
bool is_nested_construction(T const& t)
{
  return detail::nc_detect_type<T>()(t);
}

} // namespace stapl

#endif // STAPL_VIEWS_IS_NESTED_CONSTRUCTION_HPP

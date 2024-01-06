/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_LOCALIZATION_HPP
#define STAPL_PARAGRAPH_LOCALIZATION_HPP

#include <stapl/views/proxy/accessor.hpp>
#include <stapl/views/proxy/proxy.hpp>
#include <stapl/views/type_traits/is_view.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Function object that returns true if localization type
///   transformation (i.e., fast_view_type) is safe to apply.
/// @ingroup pgViewOps
/// @sa create_task
///
/// Primary template matches any view parameter and calls the @p is_local
/// member if the specifier is not ready only.
//////////////////////////////////////////////////////////////////////
template<typename T, bool = is_view<T>::value>
struct localizer
{
  template<typename View>
  static bool apply(View const& view)
  { return view.is_local(); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization matches proxies and redirects @p is_local
/// queries to the underlying accessor.
/// @ingroup pgViewOps
//////////////////////////////////////////////////////////////////////
template<typename T, typename Accessor>
struct localizer<proxy<T, Accessor>, false>
{
  static bool apply(proxy<T, Accessor> const& p)
  { return accessor_core_access::is_local(proxy_core_access::accessor(p)); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Specialization matches non proxies or view types.  Return true.
/// @ingroup pgViewOps
//////////////////////////////////////////////////////////////////////
template<typename T>
struct localizer<T, false>
{
  template<typename View>
  static bool apply(View const& view)
  { return true; }
};

} // namespace detail

} // namespace stapl

#endif // STAPL_PARAGRAPH_LOCALIZATION_HPP

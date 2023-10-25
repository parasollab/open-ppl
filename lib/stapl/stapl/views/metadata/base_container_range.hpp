/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_BASE_CONTAINER_RANGE_HPP
#define STAPL_VIEWS_METADATA_BASE_CONTAINER_RANGE_HPP

#include <stapl/views/view_traits.hpp>

#include <boost/range/iterator_range.hpp>

namespace stapl {

namespace metadata {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Default implementation for constructing an iterator range
///        over a view's container's local base containers.
///
/// @tparam View The view from which to create a base container range
//////////////////////////////////////////////////////////////////////
template <typename View>
struct base_container_range_impl
{
  using iterator = typename view_traits<View>::container::distribution_type::
    container_manager_type::iterator;
  using type = boost::iterator_range<iterator>;

  template <typename V>
  static type make(V&& v)
  {
    return type{ v.container().distribution().container_manager().begin(),
                 v.container().distribution().container_manager().end() };
  }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief Make an iterator range over the local base containers of a
///        view's container.
///
///        Note that the base containers ignore the view's domain and
///        mapping function.
//////////////////////////////////////////////////////////////////////
template <typename View>
auto make_base_container_range(View&& view) -> decltype(
  detail::base_container_range_impl<typename std::decay<View>::type>::make(
    view))
{
  return detail::base_container_range_impl<
    typename std::decay<View>::type>::make(view);
}

} // namespace metadata

} // namespace stapl

#endif

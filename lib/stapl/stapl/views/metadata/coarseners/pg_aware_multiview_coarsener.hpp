/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_METADATA_COARSENERS_PG_AWARE_MULTIVIEW_COARSENER_HPP
#define STAPL_VIEWS_METADATA_COARSENERS_PG_AWARE_MULTIVIEW_COARSENER_HPP

#include <stapl/views/metadata/coarseners/multiview.hpp>

namespace stapl {

template<typename T, typename ...OptionalParams>
class nested_pg_view_subview;

template<typename T, typename... OptionalParams>
class nested_parent_pg_view_subview;

template<typename View, bool isView = is_view<View>::value>
struct pg_aware_multiview_coarsener_impl2
{
  static
  typename std::decay<
    decltype(
      get<0>(multiview_coarsener<true>()(tuple<View>(std::declval<View>()))))
  >::type
  apply(View const& view)
  {
    return get<0>(multiview_coarsener<true>()(tuple<View>(view)));
  }
};

template <typename View>
struct pg_aware_multiview_coarsener_impl2<View, false>
{
  static View apply(View const& view)
  {
    return view;
  }
};

template<typename Views,
         typename = make_index_sequence<tuple_size<Views>::value>>
struct pg_aware_multiview_coarsener_impl;

template<typename ...Args, std::size_t... Indices>
struct pg_aware_multiview_coarsener_impl<
  tuple<Args...>, index_sequence<Indices...>>
{
  template<typename View>
  static
  auto apply(View const& views)
  STAPL_AUTO_RETURN((
    make_tuple(pg_aware_multiview_coarsener_impl2<Args>::apply(
      get<Indices>(views))...)
  ))
};

//////////////////////////////////////////////////////////////////////
/// @brief coarsening that doesn't apply coarsener on the views coming from
/// paragraph results
//////////////////////////////////////////////////////////////////////
struct pg_aware_multiview_coarsener
{
  template<typename Views>
  auto operator()(Views const& views) const
  STAPL_AUTO_RETURN((
    pg_aware_multiview_coarsener_impl<Views>::apply(views)
  ))
};
} // namespace stapl

#endif // STAPL_VIEWS_METADATA_COARSENERS_PG_AWARE_MULTIVIEW_COARSENER_HPP

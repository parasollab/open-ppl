/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_INDEX_VIEW_HPP
#define STAPL_VIEWS_INDEX_VIEW_HPP

#include <stapl/views/counting_view.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Helper function that creates a read-only view representing
///   the indices of the passed container, with the same logical
///   partitioning of elements on locations as that container.
///
/// @param ct The container whose domain and partition should be used
/// to create the @p index_view.
///
/// Relies on @ref counting_view implementation.
///
/// @todo Allow passing of any view. Tied to this is to adjust coarsener
/// to treat index_view and @ref counting_view as dont_care with regard
/// to locality (they can be arbitrarily aligned to any either view
/// in multiview coarsening).
//////////////////////////////////////////////////////////////////////
template<typename Container>
typename result_of::counting_view_nd<
  typename Container::gid_type,
  tuple_size<typename Container::gid_type>::value,
  view_impl::default_container_nd,
  typename Container::distribution_type
>::type
index_view(Container const& ct)
{
  using gid_type     = typename Container::gid_type;
  using dist_type    = typename Container::distribution_type;
  constexpr size_t N = tuple_size<gid_type>::value;

  return typename result_of::counting_view_nd<gid_type, N,
    view_impl::default_container_nd, dist_type>::type(
    new view_impl::counting_container<gid_type, N,
    view_impl::default_container_nd,  dist_type>(
    ct.dimensions(), ct.domain().first(), ct.distribution()));
}

} // namespace stapl

#endif // STAPL_VIEWS_INDEX_VIEW_HPP

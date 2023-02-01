/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_ROWS_VIEW_HPP
#define STAPL_VIEWS_ROWS_VIEW_HPP

#include <stapl/views/segmented_view.hpp>
#include <stapl/containers/partitions/rows_partition.hpp>

#include <iostream>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief A mapping function to map 1D domain's index to a 2D gid
///    using a predefined row index.
/// @tparam T1D One dimension index type.
/// @tparam T2D Two dimension index type.
//////////////////////////////////////////////////////////////////////
template <typename T1D, typename T2D >
struct fcol_2d {
  typedef T1D                  index_type;
  typedef T2D                  gid_type;

  fcol_2d(T1D row)
    : m_row(row)
  { }

  gid_type operator()(index_type const& p) const
  {
    return gid_type(m_row,p);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_row);
  }

private:
  T1D m_row;
};



//////////////////////////////////////////////////////////////////////
/// @brief Defines the rows_view type based on a partitioned view type
///    with a rows partitioner where element is a view over a row.
///
/// @tparam View A two-dimensional view to partition.
//////////////////////////////////////////////////////////////////////
template<typename View>
using rows_view = segmented_view<
  View,
  rows_partition<typename View::domain_type>,
  map_fun_gen1<fcol_2d<size_t, typename View::domain_type::index_type>>,
  view_impl::default_subview_creator<
    array_view<
      typename View::view_container_type,
      indexed_domain<size_t>,
      fcol_2d<size_t, typename View::domain_type::index_type>
    >
  >
>;

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct a partitioned view over a
///   2D view (i.e., matrix_view) to produce a view over the matrix's rows.
///
/// @param view View to partition.
/// @return A rows partitioned view.
//////////////////////////////////////////////////////////////////////
template<typename View>
rows_view<View> make_rows_view(View const& view)
{
  typedef typename View::domain_type                  domain_type;
  typedef rows_partition<domain_type>                 prows_type;
  typedef rows_view<View>                             view_type;
  return view_type(view, prows_type(view.domain()));
}

} // namespace stapl

#endif // STAPL_VIEWS_ROWS_VIEW_HPP

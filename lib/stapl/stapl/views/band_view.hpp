/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_VIEWS_BAND_VIEW_HPP
#define STAPL_VIEWS_BAND_VIEW_HPP

#include <stapl/views/segmented_view.hpp>
#include <stapl/containers/partitions/band.hpp>

#include <stapl/views/common_view.hpp>

#include <iostream>

namespace stapl {

namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to construct a partitioned view using an overlap
///        partitioner.
/// @ingroup band_view
//////////////////////////////////////////////////////////////////////
template<typename View>
class band_view_builder
{
  typedef typename View::domain_type                   domain_type;
  typedef band_partition<domain_type>                  pband_type;

public:
  typedef segmented_view<View,pband_type>            view_type;

  //////////////////////////////////////////////////////////////////////
  /// @see stapl::band_view
  //////////////////////////////////////////////////////////////////////
  view_type operator()(View& v, size_t l = 0, size_t r = 0)
  {
    return view_type(v, pband_type(v.domain(), l, r)
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @see stapl::band_view
  //////////////////////////////////////////////////////////////////////
  view_type operator()(View const& v, size_t l=0, size_t r=0)
  {
    return view_type(v, pband_type(v.domain(), l, r));
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  /// @brief use to examine this class
  //////////////////////////////////////////////////////////////////////
  void debug(char *msg=0)
  {
    std::cerr << "band_view_builder " << this << " : ";
    if (msg) {
      std::cerr << msg;
    }
    std::cerr << std::endl;
  }

};

} // namespace view_impl


//////////////////////////////////////////////////////////////////////
/// @brief Defines the band_view type over the given @p View type.
/// @tparam View View to partition.
//////////////////////////////////////////////////////////////////////
template<typename View>
using band_view = typename view_impl::band_view_builder<View>::view_type;

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct a band partitioned view.
///
/// The band domains are defined specifying the number of elements to
/// the left (@c l) of the diagonal, and the number of elements to the
/// right (@c r) of the diagonal. Each subdomain has size: l+1+r.
///
/// @par Example:
///     Domain to partition: [0..3]x[0..3]<br/>
///     left band (l): 1<br/>
///     right band (r): 1<br/>
///     Resulting partition: {[0..0]x[0..1],[1..1]x[0..2],[2..2]x[1..3],
///                           [3..3]x[2..3]}<br/>
///
/// @param view View to partition.
/// @param l Number of elements overlapped to the left.
/// @param c Number of elements overlapped to the right.
/// @return A band partitioned view.
//////////////////////////////////////////////////////////////////////
template<typename View>
band_view<View> make_band_view(View const& view, size_t l=0, size_t r=0)
{
  return view_impl::band_view_builder<View>()(view, l, r);
}

} // namespace stapl

#endif // STAPL_VIEWS_BAND_VIEW_HPP

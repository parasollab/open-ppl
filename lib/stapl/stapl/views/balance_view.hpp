/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_VIEWS_BALANCE_VIEW_HPP
#define STAPL_VIEWS_BALANCE_VIEW_HPP

#include <stapl/views/segmented_view.hpp>
#include <stapl/containers/partitions/balanced.hpp>

#include <stapl/views/common_view.hpp>

namespace stapl {

namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to construct a partitioned view using a balanced
///        partitioner
/// @ingroup balance_view
//////////////////////////////////////////////////////////////////////
template<typename View>
class part_balance_view :
  public common_view
{
  typedef typename View::domain_type                   domain_type;
  typedef balanced_partition<domain_type>              pbal_type;

public:
  typedef segmented_view<View,pbal_type>             view_type;

  //////////////////////////////////////////////////////////////////////
  /// @see stapl::balance_view
  //////////////////////////////////////////////////////////////////////
  view_type operator()(View& v, size_t n) {
    return view_type(v,pbal_type(domain_type(v.domain().first(),
                                             v.domain().last(),
                                             v.domain()), n) );
  }

  //////////////////////////////////////////////////////////////////////
  /// @see stapl::balance_view
  //////////////////////////////////////////////////////////////////////
  view_type operator()(View const& v, size_t n) {
    return view_type(v,pbal_type(domain_type(v.domain().first(),
                                             v.domain().last(),
                                             v.domain()), n) );
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  size_t version(void) const
  {
    return 0;
  }

  //////////////////////////////////////////////////////////////////////
  /// @internal
  //////////////////////////////////////////////////////////////////////
  bool validate()
  {
    return true;
  }
};

} // namespace view_impl

namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Defines balance_view type over View.
///
/// @tparam View to partition
//////////////////////////////////////////////////////////////////////
template<typename View>
struct balance_view
{
  typedef typename view_impl::part_balance_view<View>::view_type  type;
};

} // result_of namespace


//////////////////////////////////////////////////////////////////////
/// @brief Helper function to construct a balance_view
///
/// @param view to partition
/// @param n number of partitions
/// @return a balanced partitioned view
//////////////////////////////////////////////////////////////////////
template<typename View>
typename result_of::balance_view<View>::type
balance_view(const View& view, size_t n)
{
  return view_impl::part_balance_view<View>()(view,n);
}


} // stapl namespace

#endif /* STAPL_VIEWS_BALANCE_VIEW_HPP */

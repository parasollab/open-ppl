/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_VIEWS_EXPLICIT_VIEW_HPP
#define STAPL_VIEWS_EXPLICIT_VIEW_HPP

// APPEARS BROKEN

#include <stapl/domains/indexed.hpp>
#include <stapl/containers/partitions/explicit.hpp>

#include <stapl/views/common_view.hpp>

namespace stapl {

namespace view_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Defines a explicit partitioned view over the given @c View.
///
/// During construction the given explicit partition is used to
/// construct the subviews.
/// @tparam View Type of view to partition.
/// @tparam SubDom Type of domain used for the generated subviews.
/// @tparam Storage Type of container used to store the explicit domains.
/// @ingroup explicit_view
//////////////////////////////////////////////////////////////////////
template<typename View,
         typename SubDom=typename View::domain_type,
         typename Storage=std::vector<SubDom> >
class part_explicit_view :
  public common_view
{
  typedef SubDom                                        int_value_type;
  typedef indexed_domain<size_t>                        domain_type;
  typedef typename View::domain_type                    view_domain_type;
  typedef explicit_partition<view_domain_type,
                             SubDom, Storage>           pexpl_type;

public:

  typedef segmented_view<View,pexpl_type>             view_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor based on the @p view to partition and a
  ///        container where the explicit domains are specified (@p
  ///        doms_spec).
  //////////////////////////////////////////////////////////////////////
  view_type operator()(View const& view, Storage const& doms_spec)
  {
    return view_type(view, pexpl_type(view.domain(), doms_spec));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor based on the @p view to partition and an
  ///        explicit_partition object is used specify the explicit
  ///        domains.
  //////////////////////////////////////////////////////////////////////
  view_type operator()(View const& view, pexpl_type const& partition)
  {
    return view_type(view, partition);
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
  bool validate(void)
  {
    return true;
  }
};

} // namespace view_impl


namespace result_of {

//////////////////////////////////////////////////////////////////////
/// @brief Helper struct to determine the return type of the
///        explicit_view based on the given @c View and container type
///        of explicit domains (@c Storage).
//////////////////////////////////////////////////////////////////////
template <typename View, typename Storage>
struct explicit_view
{
  typedef typename view_impl::part_explicit_view<
          View, typename View::domain_type, Storage>::view_type type;
};

} // result_of namespace


//////////////////////////////////////////////////////////////////////
/// @brief Function to create an explicit_view based on the given @p
///        view and a container of domains.
//////////////////////////////////////////////////////////////////////
template<typename View, typename Storage>
typename result_of::explicit_view<View, Storage>::type
explicit_view(View const& view, Storage const& part_spec)
{
  return view_impl::part_explicit_view<
    View, typename View::domain_type, Storage>()(view,part_spec);
}

//////////////////////////////////////////////////////////////////////
/// @brief Function to create an explicit_view based on the given @p
///        view and an explicit_partition to represent the collection
///        of explicit domains.
//////////////////////////////////////////////////////////////////////
template<typename View, typename SubView, typename Storage>
typename result_of::explicit_view<View, Storage>::type
explicit_view(View const& view,
              explicit_partition<typename View::domain_type,
                                 SubView, Storage> const& exp_part)
{
  typedef view_impl::part_explicit_view<typename View::domain_type,
                                     SubView, Storage>  explicit_view_t;
  return explicit_view_t()(view, exp_part);
}

} // stapl namespace

#endif /* STAPL_VIEWS_EXPLICIT_VIEW_HPP */


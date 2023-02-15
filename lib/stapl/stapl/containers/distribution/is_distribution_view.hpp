/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_IS_DISTRIBUTION_VIEW_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_IS_DISTRIBUTION_VIEW_HPP

#include <stapl/views/system_view.hpp>
#include "is_distribution_view_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Used to determine if a type is an instantiation of a view-based
///   distribution specification.
//////////////////////////////////////////////////////////////////////
template <typename V, bool is_vw>
struct is_distribution_view
  : public std::false_type
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Used to determine if a type is an instantiation of a view-based
///   distribution specification.
///
/// Specialization for the case where the type is a view
//////////////////////////////////////////////////////////////////////
template <typename V>
struct is_distribution_view<V, true>
  : public is_distribution_view<typename V::view_container_type>
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Used to determine if a type is an instantiation of a view-based
///   distribution specification.
///
/// Specialization for the base case where the container of a view-based
/// distribution specification is a system_container instance, or other
/// views will be defined over other containers.
//////////////////////////////////////////////////////////////////////
template <typename V>
struct is_distribution_view<V, false>
  : public std::is_same<V, dist_view_impl::system_container>
{ };


template <typename DistributionView, typename PartitionInfoContainer>
struct view_based_partition;

template <typename DistributionView>
struct view_based_mapper;

//////////////////////////////////////////////////////////////////////
/// @brief Used to determine if the partition and mapper classes of a
///  container's distribution use a view-based distribution specification.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct is_view_based
  : public boost::mpl::false_
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Used to determine if the partition and mapper classes of a
///  container's distribution use a view-based distribution specification.
///
/// Specialization for the view-based partition class
//////////////////////////////////////////////////////////////////////
template <typename DistributionView, typename PartitionInfoContainer>
struct is_view_based<
         view_based_partition<DistributionView, PartitionInfoContainer>>
  : public boost::mpl::true_
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Used to determine if the partition and mapper classes of a
///  container's distribution use a view-based distribution specification.
///
/// Specialization for the view-based mapper class
//////////////////////////////////////////////////////////////////////
template <typename DistributionView>
struct is_view_based<view_based_mapper<DistributionView>>
  : public boost::mpl::true_
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Used to determine if the view is defined over a container that
///  is distributed using an arbitrary distribution.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct is_arbitrary_view_based
{
  template <typename View>
  bool operator()(View const*)
  { return false; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Used to determine if the view is defined over a container that
///  is distributed using an arbitrary distribution.
///
/// Specialization for the cases where the view_based_parition has been
/// provided with a container of @ref arbitrary_partition_info elements
/// specifying the arbitrary distribution.
//////////////////////////////////////////////////////////////////////
template <typename DistributionView, typename PartitionInfoContainer>
struct is_arbitrary_view_based<
         view_based_partition<DistributionView, PartitionInfoContainer>>
{
  template <typename View>
  bool operator()(View const*)
  { return true; }
};


//////////////////////////////////////////////////////////////////////
/// @brief Used to determine if the view is defined over a container that
///  is distributed using an arbitrary distribution.
///
/// Specialization for the cases where the view_based_parition is not given
/// a container of @ref arbitrary_partition_info elements, and the flag in
/// the @ref distribution_spec_view specifying the distribution is queried.
//////////////////////////////////////////////////////////////////////
template <typename DistributionView>
struct is_arbitrary_view_based<
         view_based_partition<DistributionView, int>>
{
  template <typename View>
  bool operator()(View const* view)
  { return this->operator()(view->container().get_distribution()); }

  template <typename Distribution>
  bool operator()(Distribution* dist)
  { return dist->partition().get_dist_view()->arbitrary(); }
};

template <typename Partition>
struct is_balanced_distribution
{
  bool operator()(Partition const&)
  { return false; }
};

template <typename DistributionView>
struct is_balanced_distribution<view_based_partition<DistributionView, int>>
{
  bool operator()(view_based_partition<DistributionView, int> const& part)
  {
    return part.get_dist_view()->distribution() == distribution_type::balanced;
  }
};
} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_IS_DISTRIBUTION_VIEW_HPP

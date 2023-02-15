/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_VIEW_BASED_MAPPER_HPP
#define STAPL_CONTAINERS_VIEW_BASED_MAPPER_HPP

#include <stapl/containers/distribution/specifications_fwd.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Mapper which takes the functor mapping partition ids to locations
/// from the distribution_view provided to the pContainer constructor.
///
/// The functor is user-defined, and as such no assumptions other than it
/// implementing a many-to-one mapping can be made.
///
/// This struct is used in tandem with @ref view_based_partition to allow
/// users to describe the data distribution of a pContainer using a pView.
///
/// @tparam DistributionView Type of the view describing a distribution.
//////////////////////////////////////////////////////////////////////
template <typename DistributionView>
struct view_based_mapper
{
  /// The domain of the partitions to be mapped (i.e., the domain [0, ..., p-1])
  typedef typename DistributionView::view_container_type::domain_type
    domain_type;

  /// The id of a partition.
  typedef typename domain_type::gid_type cid_type;

  /// The functor that will map the partition CIDs to locations.
  typedef typename DistributionView::view_container_type::map_func_type
    map_func_type;

  typedef location_type value_type;

protected:
  std::shared_ptr<DistributionView> m_dist_view;

  /// The function used to map a partition id (CID) to a location.
  std::shared_ptr<map_func_type> m_map_func;

public:
  view_based_mapper(domain_type const&)
  { abort("view_based_mapper(domain_type) not supported."); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the mapper instance for a distribution specified
  /// by a set of chained views.
  ///
  /// @param dist_view View-based specification of the distribution that
  /// will be used to map partition ids to location ids by this class
  //////////////////////////////////////////////////////////////////////
  view_based_mapper(std::shared_ptr<DistributionView> dist_view)
    : m_dist_view(dist_spec_impl::initialize_deferred(dist_view)),
      m_map_func(m_dist_view->container().mapfunc())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Construct the mapper instance for an arbitrary distribution
  /// specified by a container of @arb_partition_info instances.
  ///
  /// The pointer to the container holding the arbitrary partition information
  /// is ignored as it is not needed. It is passed to the constructor to
  /// allow the initialized_deferred call to be invoked with the flag to avoid
  /// attempting to initialize the mapping functions in @p dist_view because
  /// they're not @ref deferred_map instances.
  ///
  /// @param dist_view View-based specification of the distribution that
  /// will be used to map partition ids to location ids by this class
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionInfoContainer>
  view_based_mapper(std::shared_ptr<DistributionView> dist_view,
                    PartitionInfoContainer*)
    : m_dist_view(dist_spec_impl::initialize_deferred(dist_view, false)),
      m_map_func(m_dist_view->container().mapfunc())
  { }

  value_type map(std::tuple<cid_type, location_type, loc_qual> const& cid) const
  {
    stapl_assert(std::get<2>(cid) != LQ_LOOKUP,
      "view_based_mapper::map instructed to forward request.");
    return (*m_map_func)(std::get<0>(cid));
  }

  value_type map(cid_type const& cid) const
  {
    return (*m_map_func)(cid);
  }

  cid_type next(cid_type cid) const
  {
    return m_dist_view->container().domain().advance(cid, 1);
  }

  bool valid(cid_type cid) const
  {
    return m_dist_view->container().domain().contains(cid);
  }

  size_t get_num_locations(void) const
  {
    return m_dist_view->get_num_locations();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Update the domain of partition ids and the mapping function
  /// from partition id to location id with mapping information on elements
  /// to be inserted in the container.
  ///
  /// @param info vector of tuples, each of which specifies a contiguous set
  /// of GIDs, the partition id they map to, and the location id to which the
  /// partition id maps.
  //////////////////////////////////////////////////////////////////////
  template <typename Info>
  void update(Info const& info)
  {
    auto max = get<1>(info.front());
    for (auto const& elem : info)
      if (get<1>(elem) > max)
        max = get<1>(elem);
    m_dist_view->container().domain().update(max);
    m_map_func->update(info, 1);
  }
};

}
#endif


/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef TEST_SKELETONS_FAKE_HPP
#define TEST_SKELETONS_FAKE_HPP

#include <vector>
#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/partitions/block_partition.hpp>

namespace stapl {
namespace skeletons {
namespace tests {

// A class with minimal information that a container has to provide to
// a fake_view.
template <typename T, typename Partition>
struct fake_container
{
  using partition_type = Partition;
  using domain_type = typename Partition::domain_type;
  using index_type = typename domain_type::index_type;
  using value_type = T;

  partition_type m_partition;
  std::size_t m_location_id;

  partition_type partition() const { return m_partition; }
  domain_type const& domain() const { return m_partition.global_domain(); }
};

/// A class with minimal information that a view has to provide to spans.
template <typename C>
struct fake_view
{
  using domain_type = typename C::domain_type;
  using value_type = typename C::value_type;
  using index_type = typename C::index_type;
  C m_container;

  std::size_t get_num_locations() const { return 4; }
  domain_type domain() const { return m_container.domain(); }
  C container() const { return m_container; }
  std::vector<domain_type> local_domain(
    runtime::location_id loc_id, std::size_t num_locs) const
  {
    auto&& partition = this->container().partition();

    auto idx = partition.domain().first();
    idx = partition.domain().advance(idx, loc_id);

    return {partition[idx]};
  }

};

/// A class with minimal information that a spawner has to provide to spans.
struct fake_spawner
{
  std::size_t m_num_PEs, m_PE_id;

  std::size_t get_num_PEs() const { return m_num_PEs; }
  std::size_t get_PE_id() const { return m_PE_id; }
};

/// Creates a partition for the fake container in the 1D case
stapl::balanced_partition<indexed_domain<std::size_t, 1>>
make_partition(std::size_t const& dimension,
               std::size_t num_locations)
{
  return {indexed_domain<std::size_t, 1>{dimension}, num_locations};
}

/// Creates a partition for the fake container in the nD case
template <typename... T>
stapl::multiarray_impl::block_partition<
  typename default_traversal<sizeof...(T)>::type>
make_partition(stapl::tuple<T...> const& dimensions, std::size_t num_locations)
{
  using stapl::multiarray_impl::make_multiarray_size;
  using stapl::indexed_domain;
  using stapl::default_traversal;

  return {indexed_domain<
            std::size_t, sizeof...(T),
            typename default_traversal<sizeof...(T)>::type>{dimensions},
          make_multiarray_size<sizeof...(T)>()(num_locations)};
}

} // namespace tests
} // namespace skeletons

} // namespace stapl

#endif // TEST_SKELETONS_FAKE_HPP

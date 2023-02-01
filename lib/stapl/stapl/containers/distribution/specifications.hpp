/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DISTRIBUTION_SPECIFICATIONS_HPP
#define STAPL_CONTAINERS_DISTRIBUTION_SPECIFICATIONS_HPP

#include <stapl/utility/tuple.hpp>
#include <stapl/views/system_view.hpp>
#include "specifications_fwd.hpp"
#include <stapl/views/array_view.hpp>
#include "specification_functors.hpp"
#include <stapl/containers/partitions/block_partition.hpp>

namespace stapl {

inline distribution_spec<>
block(unsigned long int n, unsigned long int block_size, level lvl)
{
  using id_type           = unsigned long int;
  using mapping_view_type =
    dist_spec_impl::distribution_spec<>::mapping_view_type;

  const unsigned long int num_blocks = n/block_size + (n%block_size ? 1 : 0);

  auto map_factory =
    [num_blocks](location_type num_locs)
    {
      const unsigned long int blocks_per_loc =
        num_blocks/num_locs + (num_blocks%num_locs ? 1 : 0);

      return block_map<id_type, location_type>(blocks_per_loc);
    };

  return distribution_spec<>(
    new mapping_view_type(
      system_view(lvl),
      deferred_domain<indexed_domain<id_type>>(
        indexed_domain<id_type>(num_blocks),
        [](indexed_domain<id_type> const& dom, location_type)
          { return dom; }),
      deferred_map<block_map<id_type, location_type>,
                   decltype(map_factory), id_type, id_type>(map_factory),
      distribution_type::blocked),
    indexed_domain<id_type>(n),
    block_map<id_type, id_type>(block_size), distribution_type::blocked);
}


inline distribution_spec<>
block(unsigned long int n, unsigned long int block_size,
      std::vector<location_type> const& locs)
{
 using id_type           = unsigned long int;
 using mapping_view_type =
   dist_spec_impl::distribution_spec<>::mapping_view_type;

  const unsigned long int num_blocks =
    n / block_size + (n % block_size ? 1 : 0);

  const unsigned long int blocks_per_loc =
    num_blocks / locs.size() + (num_blocks % locs.size() ? 1 : 0);

  return distribution_spec<>(
    new mapping_view_type(
      system_view(locs),
      deferred_domain<indexed_domain<id_type>>(
        indexed_domain<id_type>(num_blocks),
        [](indexed_domain<id_type> const& dom, location_type)
          { return dom; }),
      block_map<id_type, location_type>(blocks_per_loc),
      distribution_type::blocked),
    indexed_domain<id_type>(n),
    block_map<id_type, id_type>(block_size), distribution_type::blocked);
}


inline distribution_spec<>
cyclic(unsigned long int n, level lvl)
{
  using id_type           = unsigned long int;
  using mapping_view_type =
    dist_spec_impl::distribution_spec<>::mapping_view_type;

  auto map_factory =
    [](location_type num_locs)
      { return cycle_map<id_type, location_type>(num_locs); };

  return distribution_spec<>(
    new mapping_view_type(
      system_view(lvl),
      deferred_domain<indexed_domain<id_type>>(
        indexed_domain<id_type>(n),
        [](indexed_domain<id_type> const& dom, location_type)
          { return dom; }),
      deferred_map<cycle_map<id_type, location_type>,
                   decltype(map_factory), id_type, id_type>(map_factory),
      distribution_type::cyclical),
    indexed_domain<id_type>(n),
    identity_map<id_type, id_type>(), distribution_type::cyclical);
}


inline distribution_spec<>
cyclic(unsigned long int n,
       std::vector<location_type> const& locs)
{
  using id_type           = unsigned long int;
  using mapping_view_type =
    dist_spec_impl::distribution_spec<>::mapping_view_type;

  return distribution_spec<>(
    new mapping_view_type(
      system_view(locs),
      deferred_domain<indexed_domain<id_type>>(
        indexed_domain<id_type>(n),
        [](indexed_domain<id_type> const& dom, location_type)
          -> indexed_domain<id_type>
        { return dom; }
      ),
      cycle_map<id_type, location_type>(locs.size()),
      distribution_type::cyclical),
    indexed_domain<id_type>(n),
    identity_map<id_type, id_type>(), distribution_type::cyclical);
}


inline distribution_spec<>
block_cyclic(unsigned long int n, unsigned long int block_size, level lvl)
{
  using id_type           = unsigned long int;
  using mapping_view_type =
    dist_spec_impl::distribution_spec<>::mapping_view_type;

  auto map_factory =
    [](location_type num_locs)
      { return cycle_map<id_type, location_type>(num_locs); };

  return distribution_spec<>(
    new mapping_view_type(
      system_view(lvl),
      deferred_domain<indexed_domain<id_type>>(
        indexed_domain<id_type>(n/block_size + (n%block_size?1:0)),
        [](indexed_domain<id_type> const& dom, location_type)
          { return dom; }),
      deferred_map<cycle_map<id_type, location_type>,
                   decltype(map_factory), id_type, id_type>(map_factory),
      distribution_type::block_cyclical),
    indexed_domain<id_type>(n),
    block_map<id_type, id_type>(block_size),
    distribution_type::block_cyclical);
}


inline distribution_spec<>
block_cyclic(unsigned long int n, unsigned long int block_size,
             std::vector<location_type> const& locs)
{
  using id_type           = unsigned long int;
  using mapping_view_type =
    dist_spec_impl::distribution_spec<>::mapping_view_type;

  return
    distribution_spec<>(
      new mapping_view_type(
        system_view(locs),
        deferred_domain<indexed_domain<id_type>>(
          indexed_domain<id_type>(n/block_size + (n%block_size?1:0)),
          [](indexed_domain<id_type> const& dom, location_type)
            { return dom; }),
        cycle_map<id_type, location_type>(locs.size()),
        distribution_type::block_cyclical),
      indexed_domain<id_type>(n),
      block_map<id_type, id_type>(block_size),
      distribution_type::block_cyclical);
}

namespace detail {
template<typename GID, typename Index>
struct balance_map_factory
{
protected:
  unsigned long int m_n;
public:
  balance_map_factory(unsigned long int n)
    : m_n(n)
  { }

  balance_map<1, GID, Index> operator()(location_type num_locs)
  { return balance_map<1, GID, Index>(m_n, num_locs); }

  void define_type(typer& t)
  { t.member(m_n); }
};
}

inline distribution_spec<>
balance(unsigned long int n, level lvl)
{
  using id_type           = unsigned long int;
  using mapping_view_type =
    dist_spec_impl::distribution_spec<>::mapping_view_type;

  detail::balance_map_factory<id_type, id_type> map_factory(n);

  return distribution_spec<>(
    new mapping_view_type(
      system_view(lvl),
      deferred_domain<indexed_domain<id_type>>(
        indexed_domain<id_type>(n),
        [](indexed_domain<id_type> const& dom, location_type num_locs)
          {
            return dom.size() > num_locs ?
                indexed_domain<id_type>(num_locs)
              : dom;
          }),
      identity_map<id_type, location_type>(), distribution_type::balanced),
    indexed_domain<id_type>(n),
    deferred_map<balance_map<1, id_type, id_type>,
                 detail::balance_map_factory<id_type, id_type>>(
      map_factory), distribution_type::balanced);
}


inline distribution_spec<>
balance(unsigned long int n, std::vector<location_type> const& locs)
{
  using id_type           = unsigned long int;
  using mapping_view_type =
    dist_spec_impl::distribution_spec<>::mapping_view_type;

  const auto locs_size = locs.size();

  auto map_factory =
    [n, locs_size](location_type)
      { return balance_map<1, id_type, id_type>(n, locs_size); };

  return distribution_spec<>(
    new mapping_view_type(
      system_view(locs),
      deferred_domain<indexed_domain<id_type>>(
        indexed_domain<id_type>(n > locs.size() ? locs.size() : n),
        [](indexed_domain<id_type> const& dom, location_type)
          { return dom; }),
      identity_map<id_type, location_type>(), distribution_type::balanced),
    indexed_domain<id_type>(n),
    deferred_map<balance_map<1, id_type, id_type>, decltype(map_factory)>(
      map_factory), distribution_type::balanced);
}


template <typename View>
distribution_spec<>
arbitrary(View const& part_view, level lvl)
{
  using id_type           = unsigned long int;
  using mapping_view_type =
    dist_spec_impl::distribution_spec<>::mapping_view_type;

  std::pair<id_type, id_type> min_max_gids =
    map_reduce(dist_spec_impl::extract_min_max_gid(),
               dist_spec_impl::combine_min_max_gid(),
               part_view);

  return distribution_spec<>(
    new mapping_view_type(
      system_view(lvl),
      deferred_domain<indexed_domain<id_type>>(
        indexed_domain<id_type>(part_view.size()),
        [](indexed_domain<id_type> const& dom, location_type)
          { return dom; }),
      dist_spec_impl::cb_part_to_loc<View>(part_view), true),
    indexed_domain<id_type>(min_max_gids.first, min_max_gids.second, true),
    dist_spec_impl::cb_gid_to_part<View>(part_view), true);
}


template <typename View>
distribution_spec<>
arbitrary(View const& part_view, std::vector<location_type> const& locs)
{
  using id_type           = unsigned long int;
  using mapping_view_type =
    dist_spec_impl::distribution_spec<>::mapping_view_type;

  std::pair<id_type, id_type> min_max_gids =
    map_reduce(dist_spec_impl::extract_min_max_gid(),
               dist_spec_impl::combine_min_max_gid(),
               part_view);

  return distribution_spec<>(
    new mapping_view_type(
      system_view(locs),
      deferred_domain<indexed_domain<id_type>>(
        indexed_domain<id_type>(part_view.size()),
        [](indexed_domain<id_type> const& dom, location_type)
          { return dom; }),
      dist_spec_impl::cb_part_to_loc<View>(part_view), true),
    indexed_domain<id_type>(min_max_gids.first, min_max_gids.second, true),
    dist_spec_impl::cb_gid_to_part<View>(part_view), true);
}


template <typename GIDMapFunc, typename PIDMapFunc>
distribution_spec<>
arbitrary(unsigned long int n, unsigned long int nparts,
          GIDMapFunc const &gid_to_pid,
          PIDMapFunc const &pid_to_lid,
          level lvl)
{
  using id_type           = unsigned long int;
  using mapping_view_type =
    dist_spec_impl::distribution_spec<>::mapping_view_type;

  return distribution_spec<>(
    new mapping_view_type(
      system_view(lvl),
      deferred_domain<indexed_domain<id_type>>(
        indexed_domain<id_type>(nparts),
        [](indexed_domain<id_type> const& dom, location_type)
          { return dom; }),
      pid_to_lid, true),
    indexed_domain<id_type>(n),
    gid_to_pid, true);
}


template <typename Dom, typename GIDMapFunc, typename PIDMapFunc, typename>
distribution_spec<Dom>
arbitrary(Dom const& dom, unsigned long int nparts,
          GIDMapFunc const &gid_to_pid,
          PIDMapFunc const &pid_to_lid,
          level lvl)
{
  using id_type           = unsigned long int;
  using mapping_view_type =
    typename dist_spec_impl::distribution_spec<Dom>::mapping_view_type;

  return distribution_spec<Dom>(
    new mapping_view_type(
      system_view(lvl),
      deferred_domain<indexed_domain<id_type>>(
        indexed_domain<id_type>(nparts),
        [](indexed_domain<id_type> const& loc_dom, location_type)
          { return loc_dom; }
      ),
      pid_to_lid, true),
    dom,
    gid_to_pid, true);
}


template <typename GIDMapFunc, typename PIDMapFunc>
distribution_spec<>
arbitrary(unsigned long int n, unsigned long int nparts,
          GIDMapFunc const &gid_to_pid,
          PIDMapFunc const &pid_to_lid,
          std::vector<location_type> const& locs)
{
  using id_type           = unsigned long int;
  using mapping_view_type =
    dist_spec_impl::distribution_spec<>::mapping_view_type;

  return distribution_spec<>(
    new mapping_view_type(
      system_view(locs),
      deferred_domain<indexed_domain<id_type>>(
        indexed_domain<id_type>(nparts),
        [](indexed_domain<id_type> const& dom, location_type)
          { return dom; }),
      pid_to_lid, true),
    indexed_domain<id_type>(n),
    gid_to_pid, true);
}


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Shared implementation function of @ref volumetric and
///   @ref balanced_nd distribution specifications.  Receives the container
///   size and lvl, together with a functor which provides the location
///   layout given a specified number of locations.
/// @ingroup distribution_specs
//////////////////////////////////////////////////////////////////////
template<typename Traversal, typename Size,
         typename LocLayoutGenerator, typename SystemParam>
typename md_distribution_spec<Traversal>::type
nd_impl(Size const& n, LocLayoutGenerator layout_gen, SystemParam&& sys_param)
{
  // Using constexpr instead of enum triggers ICE in GCC 4.9.1.
  enum { ndims = tuple_size<Size>::value };

  using volumetric_dist_type = md_distribution_spec<Traversal>;

  STAPL_IMPORT_DTYPE(volumetric_dist_type, traversal_type)
  STAPL_IMPORT_DTYPE(volumetric_dist_type, mapping_view_type)
  STAPL_IMPORT_DTYPE(volumetric_dist_type, partitioning_view_type)
  STAPL_IMPORT_DTYPE(volumetric_dist_type, gid_type)

  using gid_elem_type = typename tuple_element<0, gid_type>::type;
  using domain_type   = indexed_domain<gid_elem_type, ndims, traversal_type>;

  using gid_to_part_mf_type =
    balance_map<ndims, gid_type, gid_type, gid_type, gid_type>;

  using location_layout_type =
    typename homogeneous_tuple_type<ndims, location_type>::type;

  using partition_mapper_type = uniform_ndim_to_linear_map<gid_type>;

  auto mapping_view_map_factory =
    [n, layout_gen](location_type num_locs)
    {
      location_layout_type location_layout(layout_gen(num_locs));

      return partition_mapper_type(
        vs_map(min<unsigned int>(), n, location_layout),
        std::move(location_layout));
    };

  auto partitioning_view_map_factory =
    [n, layout_gen](location_type num_locs)
    {
      location_layout_type location_layout(layout_gen(num_locs));
      return gid_to_part_mf_type(n, gid_type(location_layout));
    };

  return partitioning_view_type(
    new mapping_view_type(
      system_view<gid_type, gid_type>(std::forward<SystemParam>(sys_param)),
      deferred_domain<domain_type>(
        [n, layout_gen](domain_type const&, location_type num_locs)
        {
          auto layout = layout_gen(num_locs);
          return domain_type(vs_map(min<unsigned int>(), n, layout));
        }),
      deferred_map<partition_mapper_type, decltype(mapping_view_map_factory),
                   gid_type, gid_type>(mapping_view_map_factory)),
    domain_type(n),
    deferred_map<gid_to_part_mf_type, decltype(partitioning_view_map_factory),
                 gid_type, gid_type>(partitioning_view_map_factory));
} // nd_impl()

} // namespace detail


template <typename Traversal, typename Size>
typename md_distribution_spec<Traversal>::type
volumetric(Size const& n, level lvl)
{
  return detail::nd_impl<Traversal>(
    n, multiarray_impl::make_multiarray_size<tuple_size<Size>::value>(), lvl);
}


template <typename Traversal, typename Size>
typename md_distribution_spec<Traversal>::type
volumetric(Size const& n, std::vector<location_type> const& locs)
{
  return detail::nd_impl<Traversal>(
    n, multiarray_impl::make_multiarray_size<tuple_size<Size>::value>(), locs);
}


namespace detail {
//////////////////////////////////////////////////////////////////////
/// @brief Helper functor to compute the product of the sizes of
/// the dimensions that are being partitioned in @ref sliced_volumetric.
///
/// The number of elements in the partitioned dimensions is an upper bound
/// on the number of partitions that can be created.  This is also the maximum
/// number of locations that can store a base container given the partition.
//////////////////////////////////////////////////////////////////////
template<int Index, int Last, typename PartitionDimensions, typename Size>
struct compute_max_partitions
{
  static std::size_t apply(Size const& n)
  {
    return
      get<tuple_element<Index, PartitionDimensions>::type::value>(n) *
      compute_max_partitions<
        Index+1, Last, PartitionDimensions, Size
      >::apply(n);
  }
};

template<int Last, typename PartitionDimensions, typename Size>
struct compute_max_partitions<Last, Last, PartitionDimensions, Size>
{
  static std::size_t apply(Size const& n)
  {
    return get<tuple_element<Last, PartitionDimensions>::type::value>(n);
  }
};

template<typename PartitionGID>
struct make_partition_gid
{
  template<typename... Indices>
  PartitionGID operator() (Indices... indices) const
  {
    return PartitionGID(indices...);
  }
};

} // namespace detail


template<typename Traversal, typename PartitionDimensions,
         typename Size, typename SystemParam>
typename sliced_md_distribution_spec<
  PartitionDimensions, Traversal
>::type
sliced_volumetric(Size const& n, SystemParam&& sys_param, Size const& start,
                  Size const& agg_size)
{
  // Using constexpr instead of enum triggers ICE in GCC 4.9.1.
  enum { ndims            = tuple_size<Size>::value };
  enum { n_partition_dims = PartitionDimensions::size() };

  using volumetric_dist_type =
    sliced_md_distribution_spec<PartitionDimensions, Traversal>;

  STAPL_IMPORT_DTYPE(volumetric_dist_type, mapping_view_type)
  STAPL_IMPORT_DTYPE(volumetric_dist_type, partitioning_view_type)
  STAPL_IMPORT_DTYPE(volumetric_dist_type, domain_type)
  STAPL_IMPORT_DTYPE(volumetric_dist_type, partition_domain_type)
  STAPL_IMPORT_DTYPE(volumetric_dist_type, gid_type)
  STAPL_IMPORT_DTYPE(volumetric_dist_type, partition_gid_type)

  using location_layout_type  =
    typename homogeneous_tuple_type<n_partition_dims, location_type>::type;

  using location_organizer    =
    multiarray_impl::make_multiarray_size<n_partition_dims>;

  using partition_mapper_type =
    uniform_ndim_to_linear_map<partition_gid_type, gid_type>;

  using gid_to_part_mf_type   =
    sliced_balance_map<ndims, PartitionDimensions,
                       gid_type, partition_gid_type>;

  // Set total size to the size of the container when no aggregate size
  // is specified.
  Size const& total_size {
    std::get<0>(agg_size) < std::get<0>(n) ? n : agg_size
  };

  partition_gid_type partition_size =
    tuple_ops::apply_impl(detail::make_partition_gid<partition_gid_type>(),
                          total_size,
                          PartitionDimensions());

  // Find the maximum number of partitions that can be created. Use this as
  // the number of partitions in the specification if the number of locations
  // is greater.
  std::size_t max_partitions =
    detail::compute_max_partitions<
      0, n_partition_dims-1,
      typename tuple_ops::from_index_sequence<PartitionDimensions>::type, Size
    >::apply(total_size);

  auto mapping_view_map_factory =
    [start, total_size, partition_size, max_partitions](location_type num_locs)
    {
      location_layout_type location_layout(
        location_organizer()(
          max_partitions > num_locs ? num_locs : max_partitions));

      // mapping the first gid to its cid determines the amount by which
      // cid has to be offset for the effective mapping of partitions
      // given the aggregated size of the set of containers
      gid_to_part_mf_type gmap(total_size, partition_gid_type(location_layout));

      return partition_mapper_type(
        vs_map(min<std::size_t>(), partition_size, location_layout),
        vs_map(min<std::size_t>(), gmap(start), location_layout),
        std::move(location_layout));
    };

  auto partitioning_view_map_factory =
    [total_size, max_partitions](location_type num_locs)
    {
      return gid_to_part_mf_type(total_size, partition_gid_type(
        location_layout_type(location_organizer()(
          max_partitions > num_locs ? num_locs : max_partitions))));
    };

  return partitioning_view_type(
    new mapping_view_type(
      system_view<gid_type, partition_gid_type>(
        std::forward<SystemParam>(sys_param)),
      deferred_domain<partition_domain_type>(
        [max_partitions, partition_size](partition_domain_type const&,
                                         location_type num_locs)
        {
          auto layout = location_organizer()(
              max_partitions > num_locs ? num_locs : max_partitions);
          return partition_domain_type(
            vs_map(min<std::size_t>(), partition_size, layout));
        }),
      deferred_map<partition_mapper_type, decltype(mapping_view_map_factory),
                   gid_type, partition_gid_type>(mapping_view_map_factory)),
    domain_type(n),
    deferred_map<gid_to_part_mf_type, decltype(partitioning_view_map_factory),
                 gid_type, partition_gid_type>(partitioning_view_map_factory));
}


template<typename Traversal, typename PartitionDimensions, typename Size>
typename sliced_md_distribution_spec<
  PartitionDimensions, Traversal
>::type
sliced_volumetric(Size const& n)
{
  return sliced_volumetric<Traversal, PartitionDimensions>(n, current_level);
}


template<typename LocDimensions,
         typename = make_index_sequence<tuple_size<LocDimensions>::value>>
class location_layout_reflector;


template<typename LocDimensions, std::size_t... Indices>
class location_layout_reflector<LocDimensions, index_sequence<Indices...>>
{
private:
  LocDimensions m_loc_dims;

public:
  location_layout_reflector(LocDimensions loc_dims)
    : m_loc_dims(std::move(loc_dims))
  { }

  LocDimensions operator()(size_t num_locs) const
  {
    stapl_assert(
      pack_ops::functional::multiplies_(get<Indices>(m_loc_dims)...) ==num_locs,
      "num locations doesn't conform to specified location dimensions");

    return m_loc_dims;
  }
}; // class location_layout_reflector


template <typename Traversal, typename Size, typename LocDimensions>
typename md_distribution_spec<Traversal>::type
balanced_nd(Size const& n, LocDimensions const& loc_dims, level lvl)
{
  return detail::nd_impl<Traversal>(
    n, location_layout_reflector<LocDimensions>(loc_dims), lvl);
}


template <typename Traversal, typename Size,
          typename LocDimensions, typename SystemParam>
typename md_distribution_spec<Traversal>::type
balanced_nd(Size const& n,
            LocDimensions const& loc_dims,
            SystemParam&& sys_param)
{
  return detail::nd_impl<Traversal>(
    n,
    location_layout_reflector<LocDimensions>(loc_dims),
    std::forward<SystemParam>(sys_param));
}


template <typename Traversal, typename Size, typename LocDimensions>
typename md_distribution_spec<Traversal>::type
uniform_nd(Size const& n, LocDimensions const& loc_dims, level lvl)
{
  return detail::nd_impl<Traversal>(
    n, location_layout_reflector<LocDimensions>(loc_dims), lvl);
}


template <typename Traversal, typename Size,
          typename LocDimensions, typename SystemParam>
typename md_distribution_spec<Traversal>::type
uniform_nd(Size const& n,
           LocDimensions const& loc_dims,
           SystemParam&& sys_param)
{
  return detail::nd_impl<Traversal>(
    n,
    location_layout_reflector<LocDimensions>(loc_dims),
    std::forward<SystemParam>(sys_param));
}

} // namespace stapl

#endif // STAPL_CONTAINERS_DISTRIBUTION_SPECIFICATIONS_HPP

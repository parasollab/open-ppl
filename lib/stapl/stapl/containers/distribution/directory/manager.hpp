/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_DIRECTORY_MANAGER_HPP
#define STAPL_CONTAINERS_DIRECTORY_MANAGER_HPP

#include <stapl/containers/partitions/balanced.hpp>
#include <stapl/containers/mapping/mapper.hpp>
#include <stapl/containers/distribution/is_distribution_view.hpp>

namespace stapl {

namespace directory_impl {

template <bool is_view_based>
struct manager_key_to_loc
{
  template <typename Partition, typename Mapper>
  static std::pair<location_type, loc_qual>
  apply(Partition& partition, Mapper& mapper,
        typename Partition::gid_type const& gid)
  {
    return std::make_pair(mapper.map(partition.find(gid)), LQ_CERTAIN);
  }
};

template <>
struct manager_key_to_loc<true>
{
  template <typename Partition, typename Mapper>
  static std::pair<location_type, loc_qual>
  apply(Partition& partition, Mapper& mapper,
        typename Partition::gid_type const& gid)
  {
    auto cid = partition.find(gid);
    if (std::get<2>(cid) == LQ_LOOKUP)
      return std::make_pair(std::get<1>(cid), LQ_LOOKUP);
    else
      return std::make_pair(mapper.map(std::get<0>(cid)), LQ_CERTAIN);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief A directory's manager maps GIDs to locations that
/// are responsible for knowing exact locality about that GID. The result
/// of this mapping is not necessarily the actual locality of the element.
/// This manager uses the partition and mapper information to perform
/// the translation.
///
/// @tparam Partition The container's partition
/// @tparam Mapper The container's mapper
///
/// @see container_directory
//////////////////////////////////////////////////////////////////////
template<typename Partition, typename Mapper>
class manager
{
private:
  Partition m_partition;
  Mapper    m_mapper;

public:
  using partition_type = Partition;
  using mapper_type    = Mapper;

  /// @brief GID of the container
  using gid_type       = typename Partition::gid_type;

  /// @brief Location type
  using value_type     = typename Mapper::value_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a manager with a given partition and mapper
  /// @param p The partition of the container
  /// @param m The mapper of the container
  //////////////////////////////////////////////////////////////////////
  manager(Partition const& partition, Mapper const& mapper)
    : m_partition(partition), m_mapper(mapper)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a manager for an arbitrary distribution specified by
  /// the elements of @p part_cont and represented by the @p partition and
  /// @p mapper parameters.
  ///
  /// @param p The partition of the container
  /// @param m The mapper of the container
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionContainer>
  manager(PartitionContainer const* const part_cont,
              Partition const& partition, Mapper const& mapper)
    : m_partition(partition), m_mapper(mapper)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Look up the home location (manager location) of a GID.
  /// @param x GID to lookup
  /// @return The location responsible for x's metadata.
  ///
  /// @note The mapper is defined as mapping pContainer component ids to
  ///       locations for data distribution.  This manager is implementing
  ///       a perfect mapping of home locations to storage locations as a
  ///       result, unless element migration has changed the original
  ///       distribution.
  //////////////////////////////////////////////////////////////////////
  std::pair<value_type, loc_qual> operator()(gid_type const& x) const
  {
    return manager_key_to_loc<is_view_based<Partition>::value>::
      apply(m_partition, m_mapper, x);
  }

  Partition const& partition() const
  {
    return m_partition;
  }

  Partition& partition()
  {
    return m_partition;
  }

  void partition(Partition const& p)
  {
    m_partition = p;
  }

  Mapper const& mapper() const
  {
    return m_mapper;
  }

  Mapper& mapper(void)
  {
    return m_mapper;
  }

  void mapper(Mapper const& m)
  {
    m_mapper = m;
  }

  void define_type(typer& t)
  {
    t.member(m_partition);
    t.member(m_mapper);
  }
}; // class manager

} // namespace directory_impl


//////////////////////////////////////////////////////////////////////
/// @brief Implementation of common manager with balanced partition and
/// block mapper.  Implemented directly in terms of general manager, but
/// reduces symbol length, compilation effort.  Used in @ref static_array
/// implementation.
//
/// @see container_directory manager array
//////////////////////////////////////////////////////////////////////
class basic_manager
  : public directory_impl::manager<
      balanced_partition<indexed_domain<size_t>>, mapper<size_t>>
{
private:
  using partition_t = balanced_partition<indexed_domain<size_t>>;

  // explicit namespace to disambiguate with method in base class.
  using mapper_t    = stapl::mapper<size_t>;

  using base_t      = directory_impl::manager<partition_t, mapper_t>;

public:
  basic_manager(partition_t const& p, mapper_t const& m)
    : base_t(p, m)
  { }

  void define_type(typer& t)
  {
    t.base<base_t>(*this);
  }
}; // class basic_manager

} // namspace stapl

#endif // STAPL_CONTAINERS_DIRECTORY_MANAGER_HPP

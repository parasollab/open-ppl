/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CONTAINER_DIRECTORY_HPP
#define STAPL_CONTAINERS_CONTAINER_DIRECTORY_HPP

#include <stapl/containers/distribution/directory/manager.hpp>
#include <stapl/containers/distribution/directory/registry.hpp>
#include <stapl/utility/directory.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief The container directory is responsible for distributed metadata for
/// GIDs. It knows in which location GIDs
/// reside. It also provides methods to invoke arbitrary functors on the
/// location of GIDs without requiring external entities to know exact locality
/// information.
///
/// The directory can be seen as the global equivalent of @ref
/// container_manager.
///
/// @tparam Partition The partition used by the container
/// @tparam Mapper The mapper used by the container
/// @tparam Manager A function object that maps GIDs to locations that
/// are responsible for knowing exact locality about that GID. The
/// default manager @ref directory_impl::manager, which uses the partition
/// and mapper information to perform the mapping.
/// @tparam Registry Storage that maps GIDs to locations. This is the exact
/// locality information for a GID. Default registry is @ref
/// directory_registry.
//////////////////////////////////////////////////////////////////////
template<typename Partition,
         typename Mapper,
         typename Manager  = use_default,
         typename Registry = use_default>
class container_directory
  : public ::stapl::directory<
      typename Partition::value_type::index_type,
      use_default,
      typename select_parameter<
        Manager, directory_impl::manager<Partition, Mapper>
      >::type,
      typename select_parameter<
        Registry,
        directory_registry<
          typename directory_impl::manager<Partition, Mapper>::gid_type>
      >::type
    >
{
public:
  typedef Partition                                            partition_type;
  typedef Mapper                                               mapper_type;
  /// The GID type
  typedef typename partition_type::value_type::index_type      key_type;
  /// The location type
  typedef typename mapper_type::value_type                     value_type;

private:
  typedef typename ::stapl::directory<
    typename Partition::value_type::index_type,
    use_default,
    typename select_parameter<
      Manager, directory_impl::manager<Partition, Mapper>
    >::type,
    typename select_parameter<
      Registry,
      directory_registry<
        typename directory_impl::manager<Partition, Mapper>::gid_type>
    >::type
   > base_t;

  STAPL_IMPORT_TYPE(typename base_t, registry_type)

public:
  STAPL_IMPORT_TYPE(typename base_t, manager_type)
  //////////////////////////////////////////////////////////////////////
  /// @brief Instantiate the directory with a container's partition
  /// and mapper.
  /// @param partition The container's partition object
  /// @param mapper The container's mapper object.
  //////////////////////////////////////////////////////////////////////
  container_directory(partition_type const& partition,
                      mapper_type const& mapper)
    : base_t(manager_type(partition, mapper),
             registry_type(manager_type(partition, mapper)))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a directory with an explicit arbitrary distribution
  /// specification. The partition and mapping information specifies the
  /// original distribution, but does not necessarily correlate to future
  /// insertions or deletions.
  ///
  /// @param part_cont Container of @ref arb_partition_info elements that
  /// specifies an arbitrary distribution
  /// @param partition The container's partition
  /// @param mapper The container's mapper
  ///////////////////////////////////////////////////////////////////////
  template <typename PartitionContainer>
  container_directory(PartitionContainer const* const part_cont,
                      partition_type const& partition,
                      mapper_type const& mapper)
    : base_t(manager_type(part_cont, partition, mapper),
             registry_type(manager_type(part_cont, partition, mapper)))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the partition for the container.
  //////////////////////////////////////////////////////////////////////
  partition_type const& partition() const
  {
    // The generic directory nomenclature for the gid (aka key) to location
    // mapping functor is key_mapper.  This is the manager object we passed
    // it at construction.
    return this->key_mapper().partition();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the partition for the container.
  //////////////////////////////////////////////////////////////////////
  partition_type& partition(void)
  {
    // The generic directory nomenclature for the gid (aka key) to location
    // mapping functor is key_mapper.  This is the manager object we passed
    // it at construction.
    return this->key_mapper().partition();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the mapper for the container.
  //////////////////////////////////////////////////////////////////////
  mapper_type const& mapper(void) const
  {
    return this->key_mapper().mapper();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the mapper for the container.
  //////////////////////////////////////////////////////////////////////
  mapper_type& mapper(void)
  {
    return this->key_mapper().mapper();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Overwrite the partition and mapper members of the manager and
  /// registry with the values provided.
  ///
  /// Redistribution requires use of view-based partition and mapper classes,
  /// which have types that are independent of the distribution they implement.
  /// This allows the use of is_same to guard the method.
  ///
  /// @param partition View-based partition of the new data distribution.
  /// @param mapper View-based mapper of the new data distribution.
  //////////////////////////////////////////////////////////////////////
  void redistribute(Partition const& partition, Mapper const& mapper)
  {
    this->m_key_mapper.partition(partition);
    this->m_key_mapper.mapper(mapper);
    this->m_registry.reset_home(partition, mapper);
    this->advance_epoch();
  }
}; // class container_directory

} // namespace stapl

#endif // STAPL_CONTAINERS_CONTAINER_DIRECTORY_HPP

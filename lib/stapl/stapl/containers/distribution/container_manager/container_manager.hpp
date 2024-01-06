/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CONTAINER_MANAGER_HPP
#define STAPL_CONTAINERS_CONTAINER_MANAGER_HPP

#include "container_manager_base.hpp"
#include "local_partition_info.hpp"
#include <stapl/containers/distribution/distributor.hpp>
#include <stapl/containers/distribution/container_manager/ordering/base_container_ordering.hpp>
#include "registry/interval.hpp"

namespace stapl {

namespace cm_impl {
//////////////////////////////////////////////////////////////////////
/// @brief Helper class used to retrieve the second element from a pair
///
/// @tparam T Pair type
/// @todo Replace usage in associative containers or move this class there.
////////////////////////////////////////////////////////////////////////
template<typename T>
struct get_second
{
  typedef typename T::second_type result_type;

  result_type operator()(T const& t) const
  {
    return t.second;
  }
};
} // namespace cm_impl


//////////////////////////////////////////////////////////////////////
/// @brief The container manager is responsible for local metadata for
/// GIDs. It knows in which local base containers GIDs
/// reside. It also provides methods to invoke base container
/// methods on specific GIDs, abstracting out the need for
/// external classes to know exactly in which base container a
/// GID is.
///
/// The container manager can be seen as the local equivalent of @ref
/// container_directory.
///
/// @tparam BContainer The base container class.
/// @todo Why is the storage of the containers public?
/// @todo For completeness, implement a const_invoke that does not have a
/// return value.
//////////////////////////////////////////////////////////////////////
template<typename BContainer, typename Registry =
           interval_container_registry<BContainer>>
class container_manager
  : public container_manager_base<Registry>
{
public:
  typedef BContainer                                       base_container_type;
  typedef Registry                                         storage_type;

  STAPL_IMPORT_TYPE(typename base_container_type, cid_type)
  STAPL_IMPORT_TYPE(typename base_container_type, gid_type)

  STAPL_IMPORT_TYPE(typename storage_type, iterator)
  STAPL_IMPORT_TYPE(typename storage_type, const_iterator)

  /// @brief Type of base container ordering that will be enforced
  typedef base_container_ordering                          ordering_type;
  typedef typename ordering_type::ptr_bcontainer_type      ptr_bcontainer_type;

protected:
  STAPL_IMPORT_TYPE(typename base_container_type, value_type)

public:
  /// @brief Handle ordering amongst base containers across all locations
  ordering_type  m_ordering;

  container_manager(void) = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that initializes the initial metadata for base
  /// containers.
  /// @param partition Partition object that translates GIDs to CIDs
  /// @param mapper Mapper object that translates CIDs to locations.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper>
  container_manager(Partition const& partition, Mapper const& mapper)
  {
    typedef typename base_container_type::domain_type domain_t;

    this->init(
      partition, mapper,
      [](domain_t const& domain, size_t cid)
        { return new base_container_type(domain, cid); },
      m_ordering);
  }

  template<typename Partition, typename Mapper>
  container_manager(Partition const& partition, Mapper const& mapper,
                    boost::mpl::false_)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that initializes the initial metadata for base
  /// containers and populates containers with a default value.
  /// @param partition Partition object that translates GIDs to CIDs
  /// @param mapper Mapper object that translates CIDs to locations.
  /// @param default_value The initial value to populate the containers
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper>
  container_manager(Partition const& partition, Mapper const& mapper,
                    value_type const& default_value)
  {
    typedef typename base_container_type::domain_type domain_t;

    value_type const* value_ptr = &default_value;

    this->init(
      partition, mapper,
      [=](domain_t const& domain, size_t cid)
        { return new base_container_type(domain, cid, *value_ptr); },
      m_ordering
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs the set of base containers required by the
  /// arbitrary distribution specified by the elements of @p part_cont.
  ///
  /// The container is provided along with the partition and mapper classes
  /// to allow construction of base containers by reading directly from the
  /// container. The partition and mapper parameters are still required by
  /// the base class and manager to allow for later retrieval of elements.
  ///
  /// @param part_cont Container of @ref arb_partition_info elements that
  /// specifies an arbitrary distribution
  /// @param partition Partition object that captures the GID to partition id
  /// mapping of the @p part_cont elements.
  /// @param mapper Mapper object that captures the partition id to location id
  /// mapping of the @p part_cont elements.
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionContainer, typename Partition, typename Mapper>
  container_manager(PartitionContainer const* const part_cont,
                    Partition const& partition, Mapper const& mapper,
                    typename std::enable_if<
                      std::is_same<typename PartitionContainer::value_type,
                        arbitrary_partition_info>::value>::type* = 0)
  {
    typedef typename base_container_type::domain_type domain_t;

    this->init(
      part_cont,
      [](domain_t const& domain, size_t cid)
        { return new base_container_type(domain, cid); },
      m_ordering);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs the set of base containers required by the
  /// arbitrary distribution specified by the elements of @p part_cont. All
  /// elements constructed in the base containers will be initialized to
  /// @p default_value.
  ///
  /// The container is provided along with the partition and mapper classes
  /// to allow construction of base containers by reading directly from the
  /// container. The partition and mapper parameters are still required by
  /// the base class and manager to allow for later retrieval of elements.
  ///
  /// @param part_cont Container of @ref arb_partition_info elements that
  /// specifies an arbitrary distribution
  /// @param partition Partition object that captures the GID to partition id
  /// mapping of the @p part_cont elements
  /// @param mapper Mapper object that captures the partition id to location id
  /// mapping of the @p part_cont elements
  /// @param default_value Value assigned to each element stored in a base
  /// container
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionContainer, typename Partition, typename Mapper>
  container_manager(PartitionContainer const* const part_cont,
                    Partition const& partition, Mapper const& mapper,
                    value_type const& default_value,
                    typename std::enable_if<
                      std::is_same<typename PartitionContainer::value_type,
                        arbitrary_partition_info>::value>::type* = 0)
  {
    typedef typename base_container_type::domain_type domain_t;

    value_type const* value_ptr = &default_value;

    this->init(
      part_cont,
      [=](domain_t const& domain, size_t cid)
        { return new base_container_type(domain, cid, *value_ptr); },
      m_ordering
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor used by associative containers to avoid creation
  /// of base containers until elements are inserted into the container.
  ///
  /// Associative containers work by distributing the domain of the possible
  /// GIDs of elements that might be stored in the container.  In order to avoid
  /// creating a large number of empty base containers that will not be used
  /// the constructor here does nothing.
  ///
  /// @param part_cont Container of @ref arb_partition_info elements that
  /// specifies an arbitrary distribution
  /// @param partition Partition object that captures the GID to partition id
  /// mapping of the @p part_cont elements
  /// @param mapper Mapper object that captures the partition id to location id
  /// mapping of the @p part_cont elements
  //////////////////////////////////////////////////////////////////////
  template<typename PartitionContainer, typename Partition, typename Mapper>
  container_manager(PartitionContainer const* const part_cont,
                    Partition const& partition, Mapper const& mapper,
                    boost::mpl::false_)
  { }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Encapsulates functionality used by assignment operator and copy
  /// constructor to perform deep copy of base container pointers stored in
  /// @p m_bcontainers.
  //////////////////////////////////////////////////////////////////////
  void clone_bcontainers(container_manager const& other)
  {
    this->clone_apply(
      other,
      [this](base_container_type* a, base_container_type* b)
        { m_ordering.replace(a, b); });
  }

public:
  container_manager(container_manager const& other)
    : m_ordering(other.m_ordering)
  {
    this->clone_bcontainers(other);
  }

  container_manager& operator=(container_manager const& other)
  {
    m_ordering = other.m_ordering;

    this->clone_bcontainers(other);

    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Performs redistribution of the container elements into a new
  /// set of bContainers on possibly different locations to match the
  /// distribution specified by the partition and mapper provided.
  /// @param partition provides methods necessary to map element GIDs to
  /// the ids of the partitions (bContainers) that will store them.
  /// @param mapper provides the methods necessary to map partition ids to
  /// the ids of the locations that will store the partitions.
  ///
  /// The method is only available when view-based partition and mapper
  /// instances are used.  The distributor instance is given the partition
  /// and mapper information in the constructor.  The distributor function
  /// operator is given the current set of bContainers and their ordering in
  /// order to perform the redistribution.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper>
  void redistribute(Partition const& partition, Mapper const& mapper,
                    typename std::enable_if<
                      is_view_based<Partition>::value
                      && is_view_based<Mapper>::value>::type* = 0)
  {
    // get local base container ids
    typedef typename Partition::value_type::index_type      index_type;
    typedef boost::icl::interval_set<index_type>            interval_set_type;
    typedef typename boost::icl::interval<index_type>::type interval_type;

    // fields are domain, partition id, destination location, source location
    typedef std::tuple<
      interval_set_type, typename Mapper::cid_type,
      location_type, std::vector<location_type>>            bc_info_type;

    typedef cm_impl::distributor<
      typename storage_type::storage_type, bc_info_type>    distributor_type;

    distributor_type d(this->m_intervals, m_ordering);
    d(partition, mapper);

    m_ordering.m_total_num_bc = partition.size();
    m_ordering.m_is_ordered   = true;
    d.advance_epoch();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the global rank of this base container in the pContainer
  /// @todo Logically should be const-qualified.
  //////////////////////////////////////////////////////////////////////
  size_t rank(base_container_type* bc)
  {
    return m_ordering.get_rank(bc);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return which base container a specified GID is in
  /// @todo Logically should be const-qualified.
  //////////////////////////////////////////////////////////////////////
  cid_type within(gid_type const& gid)
  {
    return rank(&*(this->find(gid)));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an element (container element) to this base container
  /// manager.
  /// Updates metadata accordingly.
  /// @param gid GID of the element to be added.
  /// @param val Value of the element to be added.
  /// @todo Evaluate when it's possible to add an element to an already
  /// existing base container
  /// @todo Create a new base container ID from the CIDs view (cids info)
  /// @todo Use the invoke interface to call set_element
  /// @todo Determine which chunk of the base container is smallest to move
  /// @todo This code is not tested is most certainly out of date.
  ///    with general data distributions needs to be written.
  //////////////////////////////////////////////////////////////////////
  void add_element(gid_type const& gid, value_type const& val)
  {
    abort("Add Element Called");

#if 0
    ***see todo***
    cid_type new_cid = m_bcontainers.size() > 0
         ? std::max_element(m_containers.begin(),
                            m_containers.end())->first + 1
         : cid_type();

    // base_container_type* bc = new base_container_type(
    //   typename base_container_type::domain_type(gid, gid)
    // );

    // interval_type range = boost::icl::construct<interval_type>(
    //          gid, gid, boost::icl::interval_bounds::closed());

    // m_bcontainers.insert(std::make_pair(range, bc));

    // bc->set_element(gid, val);

    invoke(gid, (void (base_container_type::*)(gid_type const&,
    typename base_container_type::value_type const&))
    &base_container_type::set_element, gid, val);
#endif
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Removes an element from this base container
  ///  manager. Updates metadata accordingly.
  /// @param gid GID of the element to be removed.
  /// @todo This code is not tested is most certainly out of date.
  ///    with general data distributions needs to be written.
  //////////////////////////////////////////////////////////////////////
  void remove_element(gid_type const& gid)
  {
    abort("Remove Element Called");
#if 0
     ***see todo***

     stapl_assert(contains(gid),
                 "removing an invalid element in base container manager");

    base_container_type* bc = m_bcontainers.find(gid)->second;

    // remove this base container if it only has one element
    if (bc->size() == 1) {
      m_bcontainers.erase(gid);
      m_ordering.remove(bc);
      delete bc;
      return;
    }

    // if removing first element of a base container
    if (gid == bc->domain().first()) {
      m_bcontainers.erase(gid);
      bc->truncate_head(gid);
      return;
    }

    // if removing last element of a base container
    if (gid == bc->domain().last()) {
      m_bcontainers.erase(gid);
      bc->truncate_tail(gid);
      return;
    }

    // removing an element in the middle of a base container
    // cid_type new_cid = m_containers.size() > 0
    //     ? std::max_element(m_containers.begin(),
    //                        m_containers.end())->first + 1
    //     : cid_type();
    typename base_container_type::domain_type new_domain =
      typename base_container_type::domain_type(bc->domain().advance(gid, 1),
                                                bc->domain().last());

    base_container_type* new_bc = new base_container_type(new_domain);

    stapl_assert(new_bc != NULL, "cannot allocate new base container");

    // Working directly with the raw sequential container
    typename base_container_type::container_type::iterator new_begin_it =
      bc->container().begin();
    std::advance(new_begin_it,
                 bc->domain().distance(bc->domain().first(), gid) + 1);

    std::copy(new_begin_it, bc->container().end(),
              new_bc->container().begin());

    bc->truncate_tail(gid);

    interval_type range = boost::icl::construct<interval_type>(
             new_domain.first(),
             new_domain.last(), boost::icl::interval_bounds::closed());

    m_bcontainers.insert(std::make_pair(range, new_bc));

    m_ordering.insert_after(bc, new_bc);

    m_bcontainers.erase(gid);
#endif
  }

  void clear(void)
  {
    m_ordering.clear();
    storage_type::clear();
  }
}; // class container_manager

} // namespace stapl

#endif // STAPL_CONTAINERS_CONTAINERS_MANAGER_HPP

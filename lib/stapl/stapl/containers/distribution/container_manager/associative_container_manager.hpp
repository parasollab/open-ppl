/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_ASSOCIATIVE_CONTAINER_MANAGER_HPP
#define STAPL_CONTAINERS_ASSOCIATIVE_CONTAINER_MANAGER_HPP

#include <stapl/containers/distribution/container_manager/container_manager.hpp>
#include <stapl/containers/distribution/container_manager/ordering/ordering_functors.hpp>
#include <boost/mpl/bool.hpp>

namespace stapl {

namespace cm_impl {

//////////////////////////////////////////////////////////////////////
/// @brief Functor implementing element insertion in an associative
/// base container.
//////////////////////////////////////////////////////////////////////
struct insert_element
{
  template <typename BaseContainer, typename GID, typename Value>
  void operator()(BaseContainer* bc, GID, Value&& val)
  {
    bc->insert(std::forward<Value>(val));
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor implementing the construction of associative base
/// containers, ignoring the domain of GIDs provided.
///
/// This is the base container factor provided to the @ref distributor
/// for the associative_container_manager::redistribute function.
/// Associative base containers use iterator_domain, and as such only
/// provide constructors that accept the component id.
//////////////////////////////////////////////////////////////////////
template <typename BaseContainer>
struct construct_bc_ignore_domain
{
  template <typename Domain, typename CID>
  static BaseContainer* apply(Domain const&, CID cid)
  {
    return new BaseContainer(cid);
  }
};

} // namespace cm_impl


//////////////////////////////////////////////////////////////////////
/// @brief The container manager is responsible for the local metadata of
///   the elements stored on this location.
///
///   It knows in which local base containers elements reside.
///   It also provides methods to invoke base container methods on specific
///   elements, abstracting out the need for external classes to know exactly
///   in which base container an element is stored. This container-manager only
///   supports one base-container per location,as multiple base-containers are
///   not needed for the unordered map.
///
/// @tparam BContainer The base container class.
/// @tparam Manager Maps the different elements to CIDs
/// @ingroup pmapDist
/// @todo This class is extremely similar to
/// @ref unordered_map_container_manager and should be merged.
//////////////////////////////////////////////////////////////////////
template<typename BContainer, typename Manager>
class associative_container_manager
  : public container_manager<BContainer,
             sparse_interval_container_registry<BContainer>>
{
  typedef container_manager<BContainer,
            sparse_interval_container_registry<BContainer>> base_type;

public:
  typedef BContainer                                       base_container_type;
  typedef typename BContainer::gid_type                    gid_type;
  typedef typename BContainer::cid_type                    cid_type;
  typedef typename BContainer::value_type                  value_type;

protected:
  typedef typename base_type::storage_type                 storage_type;
  typedef typename base_type::interval_type                interval_type;
  typedef typename base_type::ordering_type                ordering_type;
  typedef typename ordering_type::ptr_bcontainer_type      ptr_bcontainer_type;

  Manager                                                  m_manager;

public:
  STAPL_IMPORT_TYPE(typename storage_type, iterator)
  STAPL_IMPORT_TYPE(typename storage_type, const_iterator)

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that initializes the initial metadata for the base
  ///   container.
  /// @param partition Partition object that translates elements to CIDs
  /// @param mapper Mapper object that translates CIDs to locations.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper>
  associative_container_manager(Partition const& partition,
                                Mapper const& mapper)
    : base_type(partition, mapper, boost::mpl::false_()),
      m_manager(partition, mapper)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor that initializes the initial metadata for the base
  ///   container.
  /// @param manager A unordered_map_manager.
  //////////////////////////////////////////////////////////////////////
  associative_container_manager(Manager const& manager)
    : base_type(manager.partition(), manager.mapper(), boost::mpl::false_()),
      m_manager(manager)
  { }

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
  /// mapping of the @p part_cont elements
  /// @param mapper Mapper object that captures the partition id to location id
  /// mapping of the @p part_cont elements
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionContainer, typename Partition, typename Mapper>
  associative_container_manager(PartitionContainer const* const part_cont,
    Partition const& partition, Mapper const& mapper,
    typename std::enable_if<
      std::is_same<typename PartitionContainer::value_type,
      arbitrary_partition_info>::value>::type* = 0)
    : base_type(part_cont, partition, mapper, boost::mpl::false_()),
      m_manager(part_cont, partition, mapper)
  { }

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
         typename boost::enable_if<
           boost::mpl::and_<is_view_based<Partition>,
                            is_view_based<Mapper> > >::type* = 0)
  {
    // get local base container ids
    typedef typename Partition::value_type::index_type index_type;
    typedef boost::icl::interval_set<index_type> interval_set_type;
    typedef typename boost::icl::interval<index_type>::type interval_type;

    // fields are domain, partition id, destination location, source location
    typedef std::tuple<interval_set_type, typename Mapper::cid_type,
      location_type, std::vector<location_type>>  bc_info_type;

    typedef cm_impl::distributor<typename storage_type::storage_type,
      bc_info_type, cm_impl::insert_element,
      cm_impl::construct_bc_ignore_domain<base_container_type>>
      distributor_type;

    distributor_type d(this->m_intervals, this->m_ordering);
    d(partition, mapper);

    this->m_ordering.m_total_num_bc = partition.size();
    this->m_ordering.m_is_ordered   = true;
    d.advance_epoch();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the first GID out of all base containers on this location.
  /// @todo Probably needs to be moved to a different class/file operation
  ///   required for the domain interface.
  //////////////////////////////////////////////////////////////////////
  gid_type first(void) const
  {
    base_container_type* bc = this->m_ordering.first();

    if (bc != nullptr)
    {
      if (!bc->domain().empty())
        return bc->domain().first();
    }

    // return invalid gid
    return index_bounds<gid_type>::invalid();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the last GID out of all base containers on this location.
  /// @todo Probably needs to be moved to a different class/file operation
  ///   required for the domain interface.
  //////////////////////////////////////////////////////////////////////
  gid_type last(void) const
  {
    base_container_type* bc = this->m_ordering.last();

    if (bc != nullptr)
    {
      if (!bc->domain().empty())
        return bc->domain().last();
    }

    // return invalid gid
    return index_bounds<gid_type>::invalid();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears the storage. Override definition in @ref container_manager
  ///   to maintain domains in associative containers.
  //////////////////////////////////////////////////////////////////////
  void clear()
  {
    for (iterator it = this->begin(); it != this->end(); ++it)
      it->clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the GID at a certain distance from another GID.
  /// @param g The GID of reference
  /// @param n The distance to advance
  //////////////////////////////////////////////////////////////////////
  gid_type advance(gid_type const& g, long long n) // const
  {
    if (n == 0)
      return g;

    const iterator it = this->find(g);

    stapl_assert(it != this->end(), "gid not found");

    promise<gid_type> p;

    if (n > 0)
    {
      typedef ordering_detail::advance_fw<
        gid_type, base_container_type
      > advance_fw_t;

      advance_fw_t advfw(g, n, p);

      this->m_ordering.traverse_forward(advfw, &(*it));

    }
    else
    {
      typedef ordering_detail::advance_bw<
        gid_type, base_container_type
       > advance_bw_t;

      advance_bw_t advbw(g, n, p);

      this->m_ordering.traverse_backward(advbw, &(*it));
    }

    return p.get_future().get(); // sync_rmi() equivalent
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Compute the distance between two GIDs
  /// @param gid_a The first GID
  /// @param gid_b The second GID
  /// @return How many GIDs are between a and b
  //////////////////////////////////////////////////////////////////////
  size_t distance(gid_type const& gid_a, gid_type const& gid_b) //const
  {
    stapl_assert(contains(gid_a), "gid not found");

    const const_iterator bca_it = this->find(gid_a);
    const const_iterator bcb_it = this->find(gid_b);

    // Compare the base_container_type*
    if (bca_it == bcb_it)
      return bca_it->domain().distance(gid_a, gid_b);

    promise<size_t> p;

    ordering_detail::distance_fw_associative<gid_type, base_container_type>
      distfw(gid_a, gid_b, p);

    this->m_ordering.traverse_forward(distfw, &(*bca_it));

    return p.get_future().get(); // sync_rmi() equivalent
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns whether a GID is present in a base container for which
  /// this container_manager is responsible.
  ///
  /// Overriding @ref container_manager::contains.   Checking the domain
  ///   of the interval container is insufficient for associative containers.
  ///   Must check base containers.
  //////////////////////////////////////////////////////////////////////
  bool contains(gid_type const& g) const
  {
    for (const_iterator it = this->begin(); it != this->end(); ++it)
    {
      if (it->domain().contains(g))
        return true;
    }

    return false;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Find the first base container over all the base containers
  /// @param promise The promise that will be updated once the first is found.
  /// @note This function and find_last are needed for the distributed domain.
  //////////////////////////////////////////////////////////////////////
  template<typename Prom>
  void find_first(Prom const& promise) // const
  {
    typedef ordering_detail::find_fw<
      Prom, ordering_type, base_container_type
    > find_fw_t;

    find_fw_t findfw(promise, &(this->m_ordering));

    typedef void (ordering_type::*fn)(find_fw_t&, bc_base*, bool) const;

    if (this->m_ordering.get_location_id()!=0)
    {
      async_rmi(0, this->m_ordering.get_rmi_handle(),
                (fn) &ordering_type::traverse_forward, findfw, nullptr, false);
    }
    else
    {
      this->m_ordering.traverse_forward(findfw, this->m_ordering.first());
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Find the last base container over all the base containers
  /// @param promise The promise that will be updated once the last is found.
  /// @note This function and find_first are needed for the distributed domain.
  //////////////////////////////////////////////////////////////////////
  template<typename Prom>
  void find_last(Prom const& promise) // const
  {
    typedef ordering_detail::find_bw<
      Prom,ordering_type, base_container_type
    > find_bw_t;

    find_bw_t findbw(promise, &(this->m_ordering));

    typedef void (ordering_type::*fn)(find_bw_t&, bc_base*, bool) const;

    const size_t last_loc = this->m_ordering.get_num_locations() - 1;

    if (this->m_ordering.get_location_id() != last_loc)
    {
      async_rmi(last_loc, this->m_ordering.get_rmi_handle(),
                (fn) &ordering_type::traverse_backward, findbw, nullptr, false);
      return;
    }
    // else
    this->m_ordering.traverse_backward(findbw, this->m_ordering.last());
  }

  ///////////////////////////////////////////////////////////////////////
  /// @brief Invokes the erase function of the base container that contains
  ///   the target GID asynchronously.
  /// @param gid The GID to be erased
  ///////////////////////////////////////////////////////////////////////
  void erase(gid_type const& gid)
  {
    const iterator iter = this->find(gid);

    if (iter != this->end())
      iter->erase(gid);
  }

  ///////////////////////////////////////////////////////////////////////
  /// @brief Invokes the erase function of the base container that contains
  ///   the target GID.
  /// @param gid The GID to be erased
  /// @return The number of elements erased
  ///////////////////////////////////////////////////////////////////////
  size_t erase_sync(gid_type const& gid)
  {
    const iterator iter = this->find(gid);

    if (iter != this->end())
      return iter->erase_sync(gid);

    //else
    return 0;
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Find or insert a base container for the current location
  /// @todo The key argument is useless here, consider removing it.
  ///   only insert calls find_or_insert_bc.
  /// @return The base container either newly created or already existing.
  /// @todo figure out where the base container needs to be placed
  ///   (connected to who)
  //////////////////////////////////////////////////////////////////////
  base_container_type& find_or_insert_bc(gid_type const& key)
  {
    const iterator iter = this->find(key);

    if (iter != this->end())
      return *iter;

    // else
    typedef boost::icl::interval_set<typename Manager::gid_type>
      interval_set_type;
    std::tuple<interval_set_type, cid_type> hrange = m_manager.range_of(key);
    cid_type cid = get<1>(hrange);

    base_container_type* bc =
      this->insert_range(
        get<0>(hrange),
        [cid]() { return new base_container_type(cid); });

    cid_type last_cid = this->m_manager.partition().domain().last();
    cid_type next_cid = this->m_manager.mapper().next(cid);
    location_type next_loc;
    if (next_cid <= last_cid)
    {
      next_loc = this->m_manager.mapper().map(next_cid);
      this->m_ordering.insert(bc, cid, next_loc, next_cid);
    }
    else
    {
      next_loc = index_bounds<location_type>::invalid();
      this->m_ordering.insert(bc, cid, next_loc, next_cid);
    }

    return *bc;
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke the functor on the base container responsible for the
  /// specified @p gid, creating the base container if necessary.
  //////////////////////////////////////////////////////////////////////
  template<typename Class, typename Rtn, typename... PMFArgs, typename... Args>
  Rtn create_invoke(gid_type const& gid,
                    Rtn (Class::* pmf)(PMFArgs...),
                    Args&&... args)
  {
    return (find_or_insert_bc(gid).*pmf)(std::forward<Args>(args)...);
  }
}; // class associative_container_manager

} // namespace stapl

#endif

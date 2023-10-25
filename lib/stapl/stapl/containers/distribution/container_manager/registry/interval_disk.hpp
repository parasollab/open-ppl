/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CONTAINER_MANAGER_INTERVAL_DISK_CONTAINER_REGISTRY_HPP
#define STAPL_CONTAINERS_CONTAINER_MANAGER_INTERVAL_DISK_CONTAINER_REGISTRY_HPP

#include <boost/icl/interval_map.hpp>
#include <boost/iterator/transform_iterator.hpp>

#include "../ordering/base_container_ordering.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Base container storage that internally uses intervals based
///        off the GID to perform loopkups. This is used for out-of-core
///        (disk-based) graph processing.
/// @tparam Container Type of the base container.
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct interval_disk_container_registry
{
public:
  typedef Container                                        base_container_type;
  typedef typename base_container_type::gid_type           gid_type;
  typedef base_container_type                              mapped_type;

protected:
  typedef std::vector<mapped_type>                         storage_type;

  typedef boost::factory<base_container_type*>             factory_type;

private:
  typedef typename base_container_type::domain_type         domain_type;


public:
  typedef typename storage_type::iterator                   iterator;
  typedef typename storage_type::const_iterator             const_iterator;

  typedef typename storage_type::value_type                 value_type;

protected:
  storage_type m_intervals;

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Instantiate the base container that should be on this
  ///   location based on the balanced partition using a factory object
  ///   to construct instances.
  ///
  /// This specialization for @ref balanced_partition is able to construct
  /// the base containers without communication, and as a result is faster
  /// than the general algorithm implemented in the non-specialized init
  /// method.
  ///
  /// @param partition The container's partition object
  /// @param mapper The container's mapper object.
  /// @param bc_factory A factory object creating instances of the base
  ///   container on the heap, when passed the domain and container id.
  /// @param ignored instance of enable_if result type to select the method
  ///   when a balanced_partition is used.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper, typename BC_Factory>
  void init(Partition const& partition, Mapper const& mapper,
            BC_Factory const& bc_factory, base_container_ordering& ordering,
            typename boost::enable_if<boost::is_same<Partition,
              balanced_partition<typename Partition::value_type> > >::type* = 0)
  {
    typedef typename Mapper::cid_view_type cid_view_type;
    cid_view_type cids = mapper.components(get_location_id());

    if (!cids.domain().empty())
    {
      // register local gids
      base_container_type* prev_bc = nullptr;

      for (size_t it = cids.domain().first();
           it <= cids.domain().last();
           it = cids.domain().advance(it, 1))
      {
        typename base_container_type::domain_type domain = partition[it];

        if (!domain.empty())
        {
          // add local base containers
          base_container_type* bc = this->insert_range(it,
            domain, [bc_factory, &domain, it]()
              { return bc_factory(domain, it); }
          );

          ordering.insert_after(prev_bc, bc, it);
          prev_bc = bc;
        }
      }
    }

    ordering.m_total_num_bc = partition.size();
    ordering.m_is_ordered   = true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Instantiate all of the base containers that should be on this
  ///   location based on partition and mapping information using a factory
  ///   object to construct instances.
  /// @param partition The container's partition object
  /// @param mapper The container's mapper object.
  /// @param bc_factory A factory object creating instances of the base
  ///   container on the heap, when passed the domain and container id.
  /// @param ignored instance of disable_if result type to remove the method
  ///   from the overload set when a balanced_partition is used.
  /// @todo Add constructor to domain classes that accepts Boost interval
  /// to handle sparse domains properly.
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper, typename BC_Factory>
  void init(Partition const& partition, Mapper const& mapper,
            BC_Factory const& bc_factory, base_container_ordering& ordering,
            typename boost::disable_if<boost::is_same<Partition,
              balanced_partition<typename Partition::value_type> > >::type* = 0)
  {
    typedef typename Partition::value_type                  domain_type;
    typedef typename domain_type::index_type                index_type;
    typedef boost::icl::interval_set<index_type>            interval_set_type;
    typedef typename boost::icl::interval<index_type>::type interval_type;

    typedef tuple<
      boost::icl::interval_set<index_type>, typename Mapper::cid_type,
      location_type
    >  bc_info_type;

    std::vector<bc_info_type> bc_info =
      cm_impl::get_local_partition_info(
        cm_impl::get_partial_partition_info(partition, mapper));

    if (!bc_info.empty())
    {
      index_type last_cid = partition.domain().last();

      for (typename std::vector<bc_info_type>::iterator bc_it = bc_info.begin();
           bc_it != bc_info.end(); ++bc_it)
      {
        typename Mapper::cid_type it = get<1>(*bc_it);
        stapl_assert(get<0>(*bc_it).iterative_size() == 1,
          "base containers cannot be created for disjoint domains");

        interval_type interval(*get<0>(*bc_it).begin());
        domain_type
          domain(boost::icl::first(interval), boost::icl::last(interval));

        if (!domain.empty())
        {
          // add local base containers
          base_container_type* bc = this->insert_range(it,
            domain, [bc_factory, &domain, it]()
              { return bc_factory(domain, it); }
          );
          typename Mapper::cid_type next_cid = mapper.next(it);
          location_type next_loc;
          if (next_cid <= last_cid)
          {
            next_loc = mapper.map(next_cid);
            ordering.insert(bc, it, next_loc, next_cid);
          }
          else
          {
            next_loc = index_bounds<location_type>::invalid();
            ordering.insert(bc, it, next_loc, next_cid);
          }
        }
      }
    }

    ordering.m_total_num_bc = partition.size();
    ordering.m_is_ordered   = true;
  }

public:
  ~interval_disk_container_registry()
  {
    m_intervals.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns number of the local base containers in this base
  /// container manager.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return m_intervals.size();
  }

  storage_type& storage(void)
  { return this->m_intervals; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a begin iterator over the base containers in this base
  /// container manager.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return m_intervals.begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc begin
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return m_intervals.begin();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an end iterator over the base containers in this base
  /// container manager.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return m_intervals.end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc end
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return m_intervals.end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the base container that contains the
  ///        given GID. If not found, it returns an end iterator.
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  iterator find(gid_type const& gid)
  {
    auto it_e = m_intervals.end();
    for (auto it = m_intervals.begin(); it != it_e; ++it) {
      if (it->domain().contains(gid))
        return it;
    }
    return this->end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the base container that contains the
  ///        given GID. If not found, it returns an end iterator.
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  const_iterator find(gid_type const& gid) const
  {
    auto it_e = m_intervals.end();
    for (auto it = m_intervals.begin(); it != it_e; ++it) {
      if (it->domain().contains(gid))
        return it;
    }
    return this->end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the base container that contains the
  ///        given GID. If not found, it aborts in debug mode and is undefined
  ///        behavior in non-debug mode.
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  iterator find_expect(gid_type const& gid)
  {
    return this->find(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the base container that contains the
  ///        given GID. If not found, it aborts in debug mode and is undefined
  ///        behavior in non-debug mode.
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  const_iterator find_expect(gid_type const& gid) const
  {
    return this->find(gid);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears the storage and destroys the metadata for all
  /// local elements.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    for (typename storage_type::iterator it = m_intervals.begin();
         it != m_intervals.end(); ++it)
      it->clear();

    m_intervals.clear();
  }

  template<typename CIDType, typename Domain, typename Factory>
  base_container_type*
  insert_range(CIDType const& cid, Domain const& domain, Factory const& f)
  {
    m_intervals.push_back(base_container_type(domain, cid));

    return &m_intervals[m_intervals.size()-1];
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Clone the base containers and apply a function that takes the
  ///        old base container and the newly created one.
  /// @param other The other instance of the registry to clone.
  /// @param f Function that is applied after cloning that accepts the old
  ///          base container and then the new one.
  //////////////////////////////////////////////////////////////////////
  template<typename F>
  void clone_apply(interval_disk_container_registry const& other, F const& f)
  {
    typedef typename storage_type::const_iterator iter_t;

    for (typename storage_type::iterator it = m_intervals.begin();
         it != m_intervals.end(); ++it)
      delete (*it);

    this->m_intervals.clear();

    typename storage_type::const_iterator it = other.m_intervals.begin();
    typename storage_type::const_iterator eit = other.m_intervals.end();

    for (; it != eit; ++it)
    {
      base_container_type* bc = new base_container_type(*(*it));

      m_intervals.push_back(bc);

      f(*it, bc);
    }
  }
}; // struct interval_disk_container_registry

} // namespace stapl

#endif // STAPL_CONTAINERS_CONTAINER_MANAGER_INTERVAL_DISK_CONTAINER_REGISTRY_HPP

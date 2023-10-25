/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CONTAINER_MANAGER_INTERVAL_CONTAINER_REGISTRY_HPP
#define STAPL_CONTAINERS_CONTAINER_MANAGER_INTERVAL_CONTAINER_REGISTRY_HPP

#include <boost/functional/factory.hpp>
#include <boost/icl/interval_map.hpp>
#include <boost/iterator/transform_iterator.hpp>

#include "../ordering/base_container_ordering.hpp"

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper class used to retrieve the second element from a pair
///   with operator* applied.
/// @tparam T Pair type
/// @todo Replace ICL with something more suited for our uses to avoid
/// the ugly const_cast.
////////////////////////////////////////////////////////////////////////
template<typename T>
struct get_second_deref
{
private:
  typedef typename T::second_type                              iterator_t;

public:
  typedef typename std::iterator_traits<iterator_t>::reference result_type;

  result_type operator()(T const& t) const
  {
    return *const_cast<iterator_t>(t.second);
  }
};

} // namespace detail


template<typename Dom>
class block_partitioner;

//////////////////////////////////////////////////////////////////////
/// @brief Base container storage that internally uses intervals based
///        off the GID to perform loopkups.
/// @tparam Container Type of the base container.
/// @todo Generalize usage in @ref vector and make m_intervals private
///   instead of protected.
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct interval_container_registry
{
public:
  typedef Container                                        base_container_type;
  typedef typename base_container_type::gid_type           gid_type;
  typedef base_container_type*                             mapped_type;

protected:
  typedef boost::icl::interval_map<
     gid_type, mapped_type,
     boost::icl::partial_enricher
  >                                                        storage_type;
  typedef typename storage_type::key_type                  interval_type;
  typedef boost::factory<base_container_type*>             factory_type;

private:
  typedef detail::get_second_deref<
    typename storage_type::value_type
  >                                                         transform_type;
  typedef typename base_container_type::domain_type         domain_type;


public:
  typedef boost::transform_iterator<
    transform_type, typename storage_type::const_iterator
  >                                                         iterator;
  typedef boost::transform_iterator<
    transform_type, typename storage_type::const_iterator
  >                                                         const_iterator;
  typedef typename storage_type::value_type                 value_type;
  typedef typename storage_type::key_type                   key_type;

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
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper, typename BC_Factory>
  void init(Partition const& partition, Mapper const& mapper,
            BC_Factory const& bc_factory, base_container_ordering& ordering,
            typename std::enable_if<
              std::is_same<
                Partition, balanced_partition<typename Partition::value_type>
              >::value>::type* = 0)
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
          base_container_type* bc = this->insert_range(
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

  template<typename Partition, typename Mapper, typename BC_Factory>
  void init(Partition const& partition, Mapper const& mapper,
            BC_Factory const& bc_factory, base_container_ordering& ordering,
            typename std::enable_if<
              std::is_same<
                Partition, block_partitioner<typename Partition::value_type>
              >::value>::type* = 0)
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
          base_container_type* bc = this->insert_range(
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
            typename std::enable_if<
              !std::is_same<
                Partition, balanced_partition<typename Partition::value_type>
              >::value &&
              !std::is_same<
                Partition, block_partitioner<typename Partition::value_type>
              >::value
              >::type* = 0)
  {
    typedef typename Partition::value_type                  domain_type;
    typedef typename domain_type::index_type                index_type;

    if (is_balanced_distribution<Partition>()(partition))
    {
      typedef typename Partition::value_type domain_type;
      typedef partitioned_domain<domain_type> part_dom_type;
      part_dom_type part_dom(partition.global_domain());

      typename part_dom_type::subdomains_view_type subdomains =
        part_dom.local_subdomains();

      domain_type domain(subdomains[0]);

      if (!domain.empty())
      {
        auto it = get_location_id();
        auto last_cid = get_num_locations() - 1;
        // add local base containers
        base_container_type* bc = this->insert_range(
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

      ordering.m_total_num_bc = partition.size();
      ordering.m_is_ordered   = true;

      return;
    }

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
          base_container_type* bc = this->insert_range(
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

  //////////////////////////////////////////////////////////////////////
  /// @brief Instantiate all of the base containers that should be on this
  ///   location based on partition and mapping information contained in
  ///   @part_cont using a factory object to construct instances.
  /// @param part_cont Container of @ref arb_partition_info elements that
  /// specifies an arbitrary distribution
  /// @param bc_factory A factory object creating instances of the base
  ///   container on the heap, when passed the domain and container id.
  /// @todo Add constructor to domain classes that accepts Boost interval
  /// to handle sparse domains properly.
  //////////////////////////////////////////////////////////////////////
  template <typename PartitionContainer, typename BC_Factory>
  void init(PartitionContainer const* const part_cont,
            BC_Factory const& bc_factory, base_container_ordering& ordering)
  {
    typedef unsigned long int                               index_type;
    typedef boost::icl::interval_set<index_type>            interval_set_type;
    typedef typename boost::icl::interval<index_type>::type interval_type;

    typedef tuple<
      boost::icl::interval_set<index_type>, index_type, location_type
    >  bc_info_type;


    // Get the partition information for base containers to be created on this
    // location
    std::vector<bc_info_type> bc_info =
      cm_impl::get_local_partition_info(
        cm_impl::get_partial_partition_info(part_cont));

    index_type num_parts = part_cont->size();

    if (!bc_info.empty())
    {
      index_type last_cid = num_parts - 1;

      for (typename std::vector<bc_info_type>::iterator bc_it = bc_info.begin();
           bc_it != bc_info.end(); ++bc_it)
      {
        auto it = get<1>(*bc_it);
        stapl_assert(get<0>(*bc_it).iterative_size() == 1,
          "base containers cannot be created for disjoint domains");

        interval_type interval(*get<0>(*bc_it).begin());
        domain_type
          domain(boost::icl::first(interval), boost::icl::last(interval));

        // add local base containers
        base_container_type* bc = this->insert_range(domain,
          [bc_factory, &domain, it]()
            { return bc_factory(domain, it); }
        );

        index_type next_cid = it + 1;
        location_type next_loc;
        if (next_cid <= last_cid)
        {
          // This has the potential to be blocking.
          next_loc = (*part_cont)[next_cid].location();
          ordering.insert(bc, it, next_loc, next_cid);
        }
        else
        {
          next_loc = index_bounds<location_type>::invalid();
          ordering.insert(bc, it, next_loc, next_cid);
        }
      }
    }

    ordering.m_total_num_bc = num_parts;
    ordering.m_is_ordered   = true;
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return iterator to base container
  //////////////////////////////////////////////////////////////////////
  iterator check_domain_contains(gid_type const& gid) const
  {
    return std::find_if(
      this->begin(), this->end(),
      [&gid](base_container_type const& bc)
        { return bc.domain().contains(gid); });
  }


public:
  ~interval_container_registry()
  {
    for (typename storage_type::iterator it = m_intervals.begin();
         it != m_intervals.end(); ++it)
      delete (*it).second;

    m_intervals.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns number of the local base containers in this base
  /// container manager.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return m_intervals.iterative_size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a begin iterator over the base containers in this base
  /// container manager.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return boost::make_transform_iterator(m_intervals.begin(),
                                          transform_type());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc begin
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return boost::make_transform_iterator(m_intervals.begin(),
                                          transform_type());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an end iterator over the base containers in this base
  /// container manager.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return boost::make_transform_iterator(m_intervals.end(),
                                          transform_type());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc end
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return boost::make_transform_iterator(m_intervals.end(),
                                          transform_type());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the base container that contains the
  ///        given GID. If not found, it returns an end iterator.
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  iterator find(gid_type const& gid)
  {
    iterator dom_contains_iter = this->check_domain_contains(gid);

    return (dom_contains_iter != this->end()) ? dom_contains_iter :
      boost::make_transform_iterator(m_intervals.find(gid), transform_type());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the base container that contains the
  ///        given GID. If not found, it returns an end iterator.
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  const_iterator find(gid_type const& gid) const
  {
    iterator dom_contains_iter = this->check_domain_contains(gid);

    return (dom_contains_iter != this->end()) ? dom_contains_iter :
      boost::make_transform_iterator(m_intervals.find(gid), transform_type());
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
    stapl_assert(this->check_domain_contains(gid) != this->end(),
      "GID expected in this container manager, but is not found.");

    return boost::make_transform_iterator(
      m_intervals.find(gid), transform_type{}
    );
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
    stapl_assert(this->check_domain_contains(gid) != this->end(),
      "GID expected in this container manager, but is not found.");

    return boost::make_transform_iterator(
      m_intervals.find(gid), transform_type{}
    );
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears the storage and destroys the metadata for all
  /// local elements.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    for (typename storage_type::iterator it = m_intervals.begin();
         it != m_intervals.end(); ++it)
      (*it).second->clear();

    m_intervals.clear();
  }

  template<typename Factory>
  base_container_type*
  insert_range(gid_type const& first, gid_type const& last, Factory const& f)
  {
    base_container_type* bc_ptr = f();

    m_intervals.insert(std::make_pair(
      boost::icl::construct<interval_type>(
        first, last, boost::icl::interval_bounds::closed()),
      bc_ptr));

    return bc_ptr;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a base container that is responsible for a given
  ///        domain through the use of a factory and inserts it into the
  ///        registry.
  ///
  /// @param dom Domain of the base container. Currently unused.
  /// @param f Factory that when invoked, returns a pointer to a newly
  ///          created base container. The registry assumes ownership of
  ///          the pointer.
  /// @return Pointer to the base container that was added
  //////////////////////////////////////////////////////////////////////
  template<typename Domain, typename Factory>
  base_container_type* insert_range(Domain const& dom, Factory const& f)
  {
    return this->insert_range(dom.first(), dom.last(), f);
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
  void clone_apply(interval_container_registry const& other, F const& f)
  {
    typedef typename storage_type::const_iterator iter_t;

    for (typename storage_type::iterator it = m_intervals.begin();
         it != m_intervals.end(); ++it)
      delete (*it).second;

    this->m_intervals.clear();

    typename storage_type::const_iterator it  = other.m_intervals.begin();
    typename storage_type::const_iterator eit = other.m_intervals.end();

    for (; it != eit; ++it)
    {
      base_container_type* bc = new base_container_type(*(it->second));

      m_intervals.insert(std::make_pair(it->first, bc));

      f(it->second, bc);
    }
  }
}; // struct interval_container_registry


//////////////////////////////////////////////////////////////////////
/// @brief Base container storage that internally uses sets of intervals based
///        off the GID to perform loopkups.
/// @tparam Container Type of the base container.
/// @todo Generalize usage in @ref vector and make m_intervals private
///   instead of protected.
//////////////////////////////////////////////////////////////////////
template<typename Container>
struct sparse_interval_container_registry
{
public:
  typedef Container                                        base_container_type;
  typedef typename base_container_type::gid_type           gid_type;
  typedef base_container_type*                             mapped_type;

protected:
  typedef std::map<std::set<std::pair<gid_type, gid_type>>,
                   mapped_type>                            storage_type;
  typedef typename std::pair<gid_type, gid_type>           interval_type;
  typedef boost::factory<base_container_type*>             factory_type;

private:
  typedef detail::get_second_deref<
    typename storage_type::value_type
  >                                                         transform_type;
  typedef typename base_container_type::domain_type         domain_type;


public:
  typedef boost::transform_iterator<
    transform_type, typename storage_type::const_iterator
  >                                                         iterator;
  typedef boost::transform_iterator<
    transform_type, typename storage_type::const_iterator
  >                                                         const_iterator;
  typedef typename storage_type::value_type                 value_type;
  typedef typename storage_type::key_type                   key_type;

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
  //////////////////////////////////////////////////////////////////////
  template<typename Partition, typename Mapper, typename BC_Factory>
  void init(Partition const& partition, Mapper const& mapper,
            BC_Factory const& bc_factory, base_container_ordering& ordering,
            typename std::enable_if<
              std::is_same<
                Partition, balanced_partition<typename Partition::value_type>
              >::value>::type* = 0)
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
          base_container_type* bc = this->insert_range(
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
            typename std::enable_if<
              !std::is_same<
                Partition, balanced_partition<typename Partition::value_type>
              >::value>::type* = 0)
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
          base_container_type* bc = this->insert_range(
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


private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return iterator to base container
  //////////////////////////////////////////////////////////////////////
  iterator check_domain_contains(gid_type const& gid) const
  {
    return std::find_if(
      this->begin(), this->end(),
      [&gid](base_container_type const& bc)
        { return bc.domain().contains(gid); });
  }


public:
  ~sparse_interval_container_registry()
  {
    for (typename storage_type::iterator it = m_intervals.begin();
         it != m_intervals.end(); ++it)
      delete (*it).second;

    m_intervals.clear();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns number of the local base containers in this base
  /// container manager.
  //////////////////////////////////////////////////////////////////////
  size_t size(void) const
  {
    return m_intervals.iterative_size();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns a begin iterator over the base containers in this base
  /// container manager.
  //////////////////////////////////////////////////////////////////////
  iterator begin(void)
  {
    return boost::make_transform_iterator(m_intervals.begin(),
                                          transform_type());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc begin
  //////////////////////////////////////////////////////////////////////
  const_iterator begin(void) const
  {
    return boost::make_transform_iterator(m_intervals.begin(),
                                          transform_type());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an end iterator over the base containers in this base
  /// container manager.
  //////////////////////////////////////////////////////////////////////
  iterator end(void)
  {
    return boost::make_transform_iterator(m_intervals.end(),
                                          transform_type());
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc end
  //////////////////////////////////////////////////////////////////////
  const_iterator end(void) const
  {
    return boost::make_transform_iterator(m_intervals.end(),
                                          transform_type());
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Search through the sparse intervals to identify the interval
  /// that contains the specified gid.
  ///
  /// This method is used in place of the m_intervals.find(gid) call in the
  /// @ref interval_container_registry and is needed because the container
  /// used to store the intervals now doesn't provide a find method capable
  /// of scanning through the representation of the sparse domain associated
  /// with a base container.
  ///
  /// @param gid GID used to identify a base container.
  /// @return Iterator to the key-value pair of sparse domain and pointer to
  /// base container
  //////////////////////////////////////////////////////////////////////
  typename storage_type::iterator find_in_intervals(gid_type const& gid)
  {
    for (typename storage_type::iterator sparse_bc_dom = m_intervals.begin();
         sparse_bc_dom != m_intervals.end(); ++sparse_bc_dom)
    {
      for (std::pair<gid_type, gid_type> const& contig_bc_subdom
            : sparse_bc_dom->first)
      {
        if (contig_bc_subdom.first <= gid && gid <= contig_bc_subdom.second)
          return sparse_bc_dom;
      }
    }
    return m_intervals.end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Search through the sparse intervals to identify the interval
  /// that contains the specified gid.
  ///
  /// This method is used in place of the m_intervals.find(gid) call in the
  /// @ref interval_container_registry and is needed because the container
  /// used to store the intervals now doesn't provide a find method capable
  /// of scanning through the representation of the sparse domain associated
  /// with a base container.
  ///
  /// @param gid GID used to identify a base container.
  /// @return Iterator to the key-value pair of sparse domain and pointer to
  /// base container
  //////////////////////////////////////////////////////////////////////
  typename storage_type::const_iterator
  find_in_intervals(gid_type const& gid) const
  {
    typedef typename storage_type::const_iterator iterator_type;
    for (iterator_type sparse_bc_dom  = m_intervals.begin();
         sparse_bc_dom != m_intervals.end(); ++sparse_bc_dom)
    {
      for (std::pair<gid_type, gid_type> const& contig_bc_subdom
            : sparse_bc_dom->first)
      {
        if (contig_bc_subdom.first <= gid && gid <= contig_bc_subdom.second)
          return sparse_bc_dom;
      }
    }
    return m_intervals.end();
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the base container that contains the
  ///        given GID. If not found, it returns an end iterator.
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  iterator find(gid_type const& gid)
  {
    iterator dom_contains_iter = this->check_domain_contains(gid);

    return (dom_contains_iter != this->end()) ? dom_contains_iter :
      boost::make_transform_iterator(find_in_intervals(gid), transform_type());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns an iterator to the base container that contains the
  ///        given GID. If not found, it returns an end iterator.
  ///
  /// @param gid GID for the element in question.
  //////////////////////////////////////////////////////////////////////
  const_iterator find(gid_type const& gid) const
  {
    iterator dom_contains_iter = this->check_domain_contains(gid);

    return (dom_contains_iter != this->end()) ? dom_contains_iter :
      boost::make_transform_iterator(find_in_intervals(gid), transform_type());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Clears the storage and destroys the metadata for all
  /// local elements.
  //////////////////////////////////////////////////////////////////////
  void clear(void)
  {
    for (typename storage_type::iterator it = m_intervals.begin();
         it != m_intervals.end(); ++it)
      (*it).second->clear();

    m_intervals.clear();
  }

  template<typename Factory>
  base_container_type*
  insert_range(gid_type const& first, gid_type const& last, Factory const& f)
  {
    std::set<std::pair<gid_type, gid_type>> range;
    range.insert(std::make_pair(first, last));

    base_container_type* bc = f();

    m_intervals.insert(std::make_pair(range, bc));

    return bc;
  }

  template<typename GID, typename Factory>
  base_container_type*
  insert_range(
    typename boost::icl::interval_set<GID> const& ranges,
    Factory const& f)
  {
    std::set<std::pair<gid_type, gid_type>> range;

    typedef boost::icl::interval_set<GID> range_set;
    for (typename range_set::const_iterator r = ranges.begin();
         r != ranges.end(); ++r)
      range.insert(std::make_pair(r->lower(), r->upper()));

    base_container_type* bc = f();

    m_intervals.insert(std::make_pair(range, bc));

    return bc;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Constructs a base container that is responsible for a given
  ///        domain through the use of a factory and inserts it into the
  ///        registry.
  ///
  /// @param dom Domain of the base container. Currently unused.
  /// @param f Factory that when invoked, returns a pointer to a newly
  ///          created base container. The registry assumes ownership of
  ///          the pointer.
  /// @return Pointer to the base container that was added
  //////////////////////////////////////////////////////////////////////
  template<typename Domain, typename Factory>
  base_container_type* insert_range(Domain const& dom, Factory const& f)
  {
    return this->insert_range(dom.first(), dom.last(), f);
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
  void clone_apply(sparse_interval_container_registry const& other, F const& f)
  {
    typedef typename storage_type::const_iterator iter_t;

    for (typename storage_type::iterator it = m_intervals.begin();
         it != m_intervals.end(); ++it)
      delete (*it).second;

    this->m_intervals.clear();

    typename storage_type::const_iterator it  = other.m_intervals.begin();
    typename storage_type::const_iterator eit = other.m_intervals.end();

    for (; it != eit; ++it)
    {
      base_container_type* bc = new base_container_type(*(it->second));

      m_intervals.insert(std::make_pair(it->first, bc));

      f(it->second, bc);
    }
  }
}; // struct sparse_interval_container_registry

} // namespace stapl

#endif // STAPL_CONTAINERS_CONTAINER_MANAGER_INTERVAL_CONTAINER_REGISTRY_HPP

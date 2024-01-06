/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_VECTOR_DIRECTORY_HPP
#define STAPL_CONTAINERS_VECTOR_DIRECTORY_HPP

#include <stapl/containers/distribution/directory/manager.hpp>
#include <stapl/containers/distribution/directory/registry.hpp>
#include <stapl/utility/directory.hpp>

#include <functional>
#include <type_traits>

#include <boost/icl/discrete_interval.hpp>
#include <boost/icl/interval_set.hpp>
#include <boost/icl/type_traits/is_discrete.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief The container directory is responsible for distributed metadata for
/// GIDs. It knows in which location GIDs
/// reside. It also provides methods to invoke arbitrary functors on the
/// location of GIDs without requiring external entities to know exact locality
/// information.
///
/// This directory differs from @container_directory in that it supports
/// dynamic containers that insert and remove elements.
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
class vector_directory
  : public directory<
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

private:
  typedef directory<
    typename Partition::value_type::index_type,
    use_default,
    typename select_parameter<
      Manager, directory_impl::manager<Partition, Mapper>
    >::type,
    typename select_parameter<
      Registry, directory_registry<
        typename directory_impl::manager<Partition, Mapper>::gid_type
      >
    >::type
  > base_t;

  STAPL_IMPORT_TYPE(typename base_t, registry_type)

public:

  typedef Partition                                            partition_type;
  typedef Mapper                                               mapper_type;

  /// GID type
  typedef typename partition_type::value_type::index_type      key_type;

  /// Location type
  STAPL_IMPORT_TYPE(typename mapper_type, value_type)
  STAPL_IMPORT_TYPE(typename base_t, manager_type)

  /// Used to distinguish between insertion and deletion of elements
  enum action_type {INSERT, DELETE};


  //////////////////////////////////////////////////////////////////////
  /// @brief Associate a set of GIDs in the registry on this location with
  /// a specified location. This method should only be called on the location
  /// that is responsible for the range of GIDs according to the manager.
  ///
  /// @param keys A collection of GIDS to register, represented as a
  /// pair of GIDs defining the lower and upper boundary of a contiguous
  /// range.
  /// @param loc The location with which to associate the GIDs
  /// @todo Why is the assert commented out?
  //////////////////////////////////////////////////////////////////////
  template<typename K>
  void register_keys_impl(std::pair<K, K> const& keys, const location_type loc)
  {
    // stapl_assert(m_registry.find(key) == m_registry.end(),
    //   "directory::_register_key(): tried to register a key twice");

    this->m_registry.insert(std::make_pair(keys,loc));

    typename base_t::queues_t::iterator it = this->m_pending.begin();

    for ( ; it!=this->m_pending.end(); ++it)
    {
      if ((keys.first <= it->first) && (it->first <= keys.second))
      {
        const typename base_t::key_type key = it->first;
        typename base_t::queue_t& pending   = it->second;

        this->flush_pending(loc, key, pending);

        if (pending.empty())
          this->m_pending.erase(key);
      }
    }
  }


public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Create a directory not with an explicit partition and mapper,
  /// but with a given block size used to aggregate GIDs.
  ///
  /// This block size designates coarse blocks of GID registration in a
  /// cyclic manner. For example, if the block size is 2 and the set of
  /// locations is {L0, L1, L2}, then the manager information will be:
  ///
  ///   ([0..1], L0), ([2..3], L1), ([4..5], L2), ([6..7], L0), ...
  ///
  /// This constructor has the assumption that the manager takes a single
  /// parameter that is the block size, and the registry takes a single
  /// parameter which is the manager.
  ///
  /// @param block_size The number of GIDs that will be aggregated
  //////////////////////////////////////////////////////////////////////
  vector_directory(size_t block_size = 10)
    : base_t(manager_type(block_size),
             registry_type(manager_type(block_size)))
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Create a directory with an explicit partition and mapper. This
  /// partition and mapping information specifies the original distribution,
  /// but does not necessarily correlate to future insertions or deletions.
  ///
  /// @param partition The container's partition
  /// @param mapper The container's mapper
  ///////////////////////////////////////////////////////////////////////
  vector_directory(partition_type const& partition, mapper_type const& mapper)
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
  vector_directory(PartitionContainer const* const part_cont,
                   partition_type const& partition, mapper_type const& mapper)
    : base_t(manager_type(part_cont, partition, mapper),
             registry_type(manager_type(part_cont, partition, mapper)))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a directory with a given manager.
  ///
  /// @param manager Manager that this directory will use.
  //////////////////////////////////////////////////////////////////////
  vector_directory(manager_type const& manager)
    : base_t(manager, registry_type(manager))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Create a directory with a given manager, partitioner and mapper.
  ///
  /// @param partition Partition that this directory will use.
  /// @param mapper Mapper that this directory will use.
  /// @param manager Manager that this directory will use.
  //////////////////////////////////////////////////////////////////////
  vector_directory(partition_type const& partition,
                   mapper_type const& mapper,
                   manager_type const& manager)
    : base_t(manager, registry_type(manager))
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the partition for the container.
  //////////////////////////////////////////////////////////////////////
  partition_type const& partition(void) const
  {
    return this->key_mapper().partition();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return the partition for the container.
  //////////////////////////////////////////////////////////////////////
  partition_type& partition(void)
  {
    return this->key_mapper().partition();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the mapper for the container.
  //////////////////////////////////////////////////////////////////////
  mapper_type const& mapper(void) const
  {
    return this->key_mapper().mapper();
  }

  mapper_type& mapper(void)
  {
    return this->key_mapper().mapper();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return the manager of the directory.
  //////////////////////////////////////////////////////////////////////
  manager_type const& manager(void) const
  {
    return this->key_mapper();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Associate a set of GIDs in this directory's registry with the
  /// value of the location invoking this method.
  ///
  /// @param keys A collection of GIDS to register, represented as a
  /// pair of GIDs defining the lower and upper boundary of a contiguous
  /// range.
  //////////////////////////////////////////////////////////////////////
  template<typename Keys>
  void register_keys(Keys const& keys)
  {
    const size_t src_loc = this->get_location_id();

    typedef typename manager_type::map_result_type  map_intervals_t;
    typedef typename map_intervals_t::iterator      interval_iter_t;

    map_intervals_t map_managers = this->key_mapper()(keys);

    interval_iter_t it   = map_managers.begin();
    interval_iter_t e_it = map_managers.end();

    for (; it != e_it; ++it)
    {
      const size_t manager = it->second;

      if (manager == src_loc)
        this->register_keys_impl(it->first, src_loc);
      else
        async_rmi(manager, this->get_rmi_handle(),
          &vector_directory::template register_keys_impl<key_type>,
          it->first, src_loc
        );
    }
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Continues reverse iteration of registry intervals initiated by
  ///   @ref erase, updating the mapping of the key / location pair @p last.
  ///
  /// Recursively calls the next interval unless the current interval
  /// contains @p target_key. In that case, signal via the promise that the
  /// directory updates for this erase are complete.
  //////////////////////////////////////////////////////////////////////
  void erase_impl(key_type const& target_key,
                  std::pair<key_type, location_type> const& last,
                  promise<void> signal)
  {
    auto iter = this->m_registry.find(last.first);

    stapl_assert(iter != this->m_registry.end(), "Error in erase_impl");

    auto range    = iter->first;
    auto location = iter->second;

    this->m_registry.erase(last.first);
    this->m_registry.insert(last);

    if (boost::icl::contains(range, target_key))
    {
      signal.set_value();
      return;
    }

    const size_t mng_location =
      this->key_mapper()(boost::icl::first(range)-1).first;

    async_rmi(mng_location, this->get_rmi_handle(),
              &vector_directory::erase_impl,
              target_key, std::make_pair(boost::icl::first(range)-1, location),
              std::move(signal));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Continues forward iteration of registry intervals initiated by
  ///   @ref insert, updating the mapping of the key / location pair
  ///   @p first.
  ///
  /// Recursively calls the next interval unless the current
  /// interval contains @p target_key (the last key in container).
  /// In that case, register a new key at the end with appropriate mapping
  /// and then signal via the promise that the directory updates for this
  /// insert are complete.
  //////////////////////////////////////////////////////////////////////
  void insert_impl(key_type const& target_key,
                   std::pair<key_type, location_type> const& first,
                   promise<void> signal)
  {
    auto iter = this->m_registry.find(first.first);

    stapl_assert(iter != this->m_registry.end(), "Error in insert_impl");

    auto interval_last  = boost::icl::last(iter->first);
    auto location       = iter->second;

    this->m_registry.erase(first.first);
    this->m_registry.insert(first);

    if (interval_last == target_key)
    {
      this->register_apply(
        target_key + 1,
        std::bind([](promise<void>& s) { s.set_value(); }, std::move(signal)),
        location);
      return;
    }

    const size_t mng_location = this->key_mapper()(interval_last + 1).first;

    // else
    async_rmi(mng_location, this->get_rmi_handle(),
              &vector_directory::insert_impl, target_key,
              std::make_pair(interval_last + 1, location), std::move(signal));
  }

public:
  /////////////////////////////////////////////////////////////////////
  /// @brief Adjust gid to location mappings caused by erasure of gid @p key
  ///   from the container with current last gid @p last. Then signal completion
  ///   to @ref vector_distribution::erase via the promise @p signal.
  /////////////////////////////////////////////////////////////////////
  void erase(key_type const& key, key_type const& last, promise<void> signal)
  {
    if (this->get_location_id() != this->key_mapper()(last).first)
      return;

    const auto next_update = this->m_registry.shift_left(last, key);

    if (next_update.second == invalid_location_id)
    {
      signal.set_value();
      return;
    }

    const size_t mng_location = this->key_mapper()(next_update.first).first;

    async_rmi(mng_location, this->get_rmi_handle(),
              &vector_directory::erase_impl,
              key, next_update, std::move(signal));
   }

  /////////////////////////////////////////////////////////////////////
  /// @brief Adjust gid to location mappings caused by insertion of a value
  ///  at gid @p key from the container with current last gid @p last. Then
  ///  signal completion to @ref vector_distribution::insert via the promise
  ///  @p signal.
  /////////////////////////////////////////////////////////////////////
  void insert(key_type const& key, key_type const& last, promise<void> signal)
  {
    if (this->get_location_id() != this->key_mapper()(key).first)
      return;

    if (key == last + 1)
    {
      this->register_key(last + 1);

      signal.set_value();

      return;
    }

    const auto next_update = this->m_registry.shift_right(key);

    // Interval with insertion also has last element...
    if (next_update.first == last + 1)
    {
      this->register_apply(
        last + 1,
        std::bind([](promise<void>& s) { s.set_value(); }, std::move(signal)),
        next_update.second);
      return;
    }

    const size_t mng_location = this->key_mapper()(next_update.first).first;

    async_rmi(mng_location, this->get_rmi_handle(),
              &vector_directory::insert_impl,
              last, next_update, std::move(signal));
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return whether or not a certain GID is registered on the location
  /// invoking this method.
  /// @param key The GID in question
  //////////////////////////////////////////////////////////////////////
  bool is_registered_local(key_type const& key) const
  {
    return this->m_registry.find(key) != this->m_registry.end();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Return whether or not a certain GID is registered on every
  /// location.
  /// @param key The GID in question
  /// @todo Remove blocking behavior by implementing a method that returns
  /// a promise.
  //////////////////////////////////////////////////////////////////////
  bool is_registered(key_type const& key) const
  {
    const std::pair<location_type, loc_qual> loc_id = this->key_mapper()(key);

    if (loc_id.first == this->get_location_id() && loc_id.second == LQ_CERTAIN)
      return is_registered_local(key);
    else if (loc_id.second == LQ_CERTAIN)
      return sync_rmi(loc_id.first, this->get_rmi_handle(),
                      &vector_directory::is_registered_local, key);
    else
      return sync_rmi(loc_id.first, this->get_rmi_handle(),
                      &vector_directory::is_registered, key);
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
  template <typename NewPartition, typename NewMapper>
  void redistribute(NewPartition const& partition, NewMapper const& mapper,
                    typename std::enable_if<
                      std::is_same<NewPartition, Partition>::value
                      && std::is_same<NewMapper, Mapper>::value
                    >::type* = 0)
  {
    this->m_key_mapper.partition(partition);
    this->m_key_mapper.mapper(mapper);
    this->m_registry.reset_home(partition, mapper);
    this->advance_epoch();
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Internal method used to facilitate invocation of @p f on location
  /// where @p key is currently registered. If @p key is not registered at the
  /// expected location, retries to invoke @p f at the location where @p key-1
  /// is expected to be registered.
  ///
  /// This continues until the location with registered key has been found (in
  /// which case @p f is called via RMI on the appropriate directory location
  /// where @p key is managed) or until all positive keys have been tried.
  /// Used in @ref vector_distribution::pop_back.
  ///
  /// @param key Key whose registered location determines where the search for
  ///   execution location for @p f should be started.
  ///
  /// @param f Functor to invoke (unary, @p key passed as parameter).
  ///
  /// @sa vector_distribution::pop_back
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void decreasing_request_forward(key_type const& key, Functor&& f)
  {
    if (is_registered_local(key))
      base_t::request_forward(key, std::forward<Functor>(f));
    else if (key != 0)
      this->decreasing_invoke_where(std::forward<Functor>(f), key-1);
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke function object at the location where key is currently
  /// registered; if the key has been unregistered at its managing location,
  /// try invoking at the location where key-1 should be registered.
  ///
  /// @param f Functor to apply. Unary operator, receives key as parameter.
  /// @param key Key that may not be registered when method is invoked
  ///
  /// @sa decreasing_request_forward
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void decreasing_invoke_where(Functor&& f, key_type const& key)
  {
    using mem_fun_t = void (vector_directory::*)(key_type const&, decltype(f));

    constexpr mem_fun_t mem_fun = &vector_directory::decreasing_request_forward;

    this->invoke_at_manager(key, mem_fun, key, std::forward<Functor>(f));
  }
}; // class vector_directory

} // namespace stapl

#endif // STAPL_CONTAINERS_VECTOR_DIRECTORY_HPP

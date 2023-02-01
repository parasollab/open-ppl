/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_EDGE_CONTAINER_H
#define STAPL_PARAGRAPH_EDGE_CONTAINER_H

#include <stapl/algorithms/functional.hpp>
#include <stapl/paragraph/edge_container/edge_entry_base.hpp>
#include <stapl/paragraph/edge_container/edge_local_notifier.hpp>

#include <stapl/utility/directory.hpp>
#include <stapl/utility/down_cast.hpp>
#include <stapl/utility/use_default.hpp>


namespace stapl {

namespace detail {

template<typename T>
class edge_entry;

template<typename T>
class edge_entry_wrapper;


//////////////////////////////////////////////////////////////////////
/// @brief Holds bucket count and array of buckets for hashtable used
/// in @ref edge_storage (and eventually by @ref edge_container).
/// Encapsulated here to properly manage lifetime, so that buckets are
/// destroyed after the Boost.Intrusive unordered_set they serve.
///
/// @ingroup pgEdgeContainer
///
/// @todo Investigate using prime num buckets and size hints from
///   Boost.Intrusive unordered_set implementation.
///////////////////////////////////////////////////////////////////////
template<typename Bucket, typename Traits>
class hash_buckets
{
private:
  /// @brief Number of buckets allocated in frame as part of small
  /// container optimization (similar to std::string).
  static constexpr size_t initial_num_buckets = 16;

  using external_bucket_ptr_t = std::unique_ptr<Bucket[]>;

  /// @brief Number of buckets currently in used.
  mutable size_t                        m_num_buckets;

  /// @brief Bucket storage for the unordered set.  Boost.Intrusive containers
  /// do no memory allocation, so we must create them and pass to constructor
  /// of the value_cache. Union used to implement small container optimization,
  /// to avoid a heap allocation for paragraphs with a small number of edges.
  union {
    mutable external_bucket_ptr_t       m_buckets_ptr;
    mutable Bucket                      m_initial_buckets[initial_num_buckets];
  };

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Explicitly call constructor of union member m_initial_buckets
  /// so that buckets are initialized prior to use.
  //////////////////////////////////////////////////////////////////////
  hash_buckets(void)
    : m_num_buckets(initial_num_buckets),
      m_initial_buckets()
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Explicitly call destructor of external bucket storage if we
  /// have spilled out of the initial, in-frame set of buckets.
  //////////////////////////////////////////////////////////////////////
  ~hash_buckets()
  {
    if (m_num_buckets != initial_num_buckets)
      m_buckets_ptr.~external_bucket_ptr_t();
  }

  size_t size(void) const
  { return m_num_buckets; }

  //////////////////////////////////////////////////////////////////////
  /// @brief Allocate a new set of buckets and call rehash on the
  /// passed unordered_set.
  ///
  /// Handle corner case of in frame buckets with placement new call.
  /// In steady state case, move assignment on unique_ptr storage cleans
  /// up previously allocated buffers.
  //////////////////////////////////////////////////////////////////////
  template<typename Container>
  void grow(Container& ct) const
  {
    const size_t new_bucket_count = m_num_buckets * 2;

    external_bucket_ptr_t new_buckets(new Bucket[new_bucket_count]);

    ct.rehash(Traits(new_buckets.get(), new_bucket_count));

    if (m_num_buckets == initial_num_buckets)
      new (&m_buckets_ptr) external_bucket_ptr_t(std::move(new_buckets));
    else
      m_buckets_ptr = std::move(new_buckets);

    m_num_buckets = new_bucket_count;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns raw pointer to bucket array, whether it be stored
  /// in frame or on the heap.
  //////////////////////////////////////////////////////////////////////
  Bucket* get(void) const
  {
    if (m_num_buckets == initial_num_buckets)
      return m_initial_buckets;
    else
      return m_buckets_ptr.get();
  }
}; // class hash_buckets


//////////////////////////////////////////////////////////////////////
/// @brief Manages storage of edges of the @ref edge_container.
///
/// Callers know the intrusive nature of entries (i.e., they are responsible
/// for allocation and deallocation of entries), but the exact storage container
/// (e.g., ordered_set vs unordered_set) is abstracted.  Furthermore, the
/// interface of the underlying Boost.Intrusive container is restricted to not
/// allow access via iterators.  This allows operations that invalidate
/// iterators such as rehashing to occur without other parts of the stack
/// needing to know.
///
/// @ingroup pgEdgeContainer
///
/// @todo Could maybe avoid duplicate internal storage of number of buckets in
///   unordered_set and @ref hash_buckets via some mechanism to pass current
///   number of buckets to the latter after the former's destruction.  Custom
///   bucket traits that simply wrap @ref hash_buckets isn't safe.
///
/// @todo Get rid of mutable on m_value_cache and bucket storage.
///
/// @todo Investigate use of intrusive::cache_begin<true> to avoid cost to scan
///   of unordered_set at destruction, at cost of per insert/delete operation
///
/// @todo Investigate alternate small container strategies to reduce startup
///   costs.  Options include smaller initial bucket, adaptive bucket growth
///   strategy, etc.
//////////////////////////////////////////////////////////////////////
class edge_storage
{
public:
  /// @brief The task identifier type used by the PARAGRAPH.
  using index_type = std::size_t;

  /// @brief The edge entry type held by the value cache.
  using edge_entry_t = detail::edge_entry_base;

private:
  /// @brief The type of container used for storage of edge entries.
  using value_cache_t =
    boost::intrusive::unordered_set<
      edge_entry_t,
      boost::intrusive::constant_time_size<true>,
      boost::intrusive::power_2_buckets<true>>;

  using buckets_t       = value_cache_t::bucket_type;
  using bucket_traits_t = value_cache_t::bucket_traits;

  /// @brief Buckets used by unordered_set.
  mutable hash_buckets<buckets_t, bucket_traits_t>             m_buckets;

  /// @brief Contains an edge entry for every task that is
  /// executed or consumed on this location.
  mutable value_cache_t                                        m_value_cache;

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Called by @ref edge_container methods that increase the number
  /// of entries stored in the local cache.  If load factor is 1, resize
  /// by a power of two.
  //////////////////////////////////////////////////////////////////////
  void check_increase_size(void) const
  {
    if (m_value_cache.size() < m_buckets.size())
      return;

    m_buckets.grow(m_value_cache);
  }


public:
  edge_storage(void)
    : m_value_cache(bucket_traits_t(m_buckets.get(), m_buckets.size()))
  { }

  edge_storage& operator=(edge_storage const&) = delete;
  edge_storage(edge_storage const&) = delete;

  //////////////////////////////////////////////////////////////////////
  /// @brief Search for entry with key @p tid.  If found, return a pointer
  /// else return a nullptr.
  //////////////////////////////////////////////////////////////////////
  edge_entry_t* lookup_ptr(index_type tid) const
  {
    auto iter = m_value_cache.find(
      tid, boost::hash<std::size_t>(), detail::entry_compare());

    return iter == m_value_cache.end() ? nullptr : &(*iter);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Search for entry with key @p tid.  The entry is expected to
  /// exist; assert if it is not found.  Return a reference to the entry.
  //////////////////////////////////////////////////////////////////////
  edge_entry_t& lookup_expect(index_type tid)
  {
    auto iter = m_value_cache.find(
      tid, boost::hash<std::size_t>(), detail::entry_compare());

    stapl_assert(iter != m_value_cache.end(),
      "lookup_expect found no cache entry");

    return *iter;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if there is an entry for task @p tid in the
  /// value cache.
  //////////////////////////////////////////////////////////////////////
  bool contains(index_type tid) const
  {
    return m_value_cache.end() !=
      m_value_cache.find(
        tid, boost::hash<std::size_t>(), detail::entry_compare());
  }

  bool empty(void) const
  { return m_value_cache.empty(); }

  void erase(edge_entry_t& entry_ref) const
  {
    m_value_cache.erase_and_dispose(
      m_value_cache.iterator_to(entry_ref), detail::entry_disposer());
  }

  void clear(void)
  { this->m_value_cache.clear_and_dispose(detail::entry_disposer()); }

  bool insert(edge_entry_t& entry_ref)
  {
    this->check_increase_size();
    return m_value_cache.insert(entry_ref).second;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Check if entry exists for given task id and create one if
  /// it doesn't. Returns a reference to the entry.
  //////////////////////////////////////////////////////////////////////
  template<typename EntryFactory>
  edge_entry_t&
  lookup_or_insert(const index_type tid, EntryFactory factory)
  {
    typename value_cache_t::insert_commit_data insert_data;

    auto tmp = m_value_cache.insert_check(
      tid, boost::hash<std::size_t>(), detail::entry_compare(), insert_data);

    edge_entry_t& ret_val = tmp.second ?
      *m_value_cache.insert_commit(factory(tid), insert_data)
      : *tmp.first;

    this->check_increase_size();

    return ret_val;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Apply functor on all entries in the value cache.
  //////////////////////////////////////////////////////////////////////
  template<typename Functor>
  void for_each(Functor f) const
  {
    for (auto&& entry : m_value_cache)
      f(entry);
  }
}; // class edge_storage

} // namespace detail


using paragraph_impl::task_graph;


//////////////////////////////////////////////////////////////////////
/// @brief The edge container maintains a distributed representation of the
/// dependence (edges) for the PARAGRAPH and the various data flows they
/// represent.  It is the primary container for the PARAGRAPH infrastructure
/// which is traversed during PARAGRAPH execution as tasks complete execution
/// and notify the edge_container of their return value (if any), triggering
/// data flow to all specified successors in the system.
///
/// The edge container facilitates the unordered creation of predecessor and
/// successor tasks, allowing the edge creation to occur without any
/// synchronization between the two.
///
/// In ephemeral mode, storage for both intermediate values and the dependence
/// information is aggressively reclaimed to minimize the footprint of the
/// PARAGRAPH
///
/// @ingroup pgEdgeContainer
///
/// @todo @p receive_signal can probably be merged with @p receive_value with a
/// little further refactoring.  The same goes for @p setup_flow and
/// @p setup_signal_flow.
///
/// @todo Simplify the lifetime of the edge_container.  Previously it was
/// managed independently of the PARAGRAPH because uses of the edge values could
/// escape outside the PARAGRAPH.  This is no longer the case, and it is
/// probably best for the PARAGRAPH and the edge_container to share lifetimes
/// or even be combined via inheritance or making the edge_container a member
/// of the PARAGRAPH.
///
/// @todo The numerous booleans could be consolidated into a bit field to be a
/// little more memory conscious.
///
/// @todo - boomerang requests originating from @p add_producer causes greater
/// latency to evict, and can be done (I think) if migration is turned off.
/// Need more typing info to perhaps change policy if migration off..
///
/// @todo There's a redundant lookup of the entry in add_remote_notifier and
/// add_remote_signal_notifier on producer locations when setup_flow calls this
/// to add the remote notifier specifically for migration.  Refactor cleanly if
/// possible to avoid.
///
/// @todo Consider more aggressive removal of entries from value cache prior to
///  eviction, but when initialization in the neighborhood is complete. Requires
///  using graph traversal to clean up in persistent case.
//////////////////////////////////////////////////////////////////////
class edge_container final
  : public directory<
      std::size_t, try_transmitter,
      boost::function<std::pair<location_type, loc_qual>(size_t)>,
      use_default, true>,
    public detail::edge_storage
{
public:
  /// @brief The successor notifier type passed to the @p edge_container
  /// by consumption requests originating in @p task_graph_impl::add_task.
  using local_notifier_t = detail::edge_local_notifier_base;

private:
  /// @brief The notifier type used for signal consumers.
  using signal_notifier_t = boost::function<void (executor_base&)>;

  using directory_t =
    directory<
      index_type, try_transmitter,
      boost::function<std::pair<location_type, loc_qual>(size_t)>,
      use_default, true>;

  /// @brief Pointer to PARAGRAPH associated with the edge_container.
  paragraph_impl::task_graph*                            m_tg_ptr;

  /// @brief Used to delay deletion when setting a data flow in an edge entry
  /// until it can flush all notifications and return, allowing the
  /// edge_container to properly evict it before deleting itself.
  bool                                                   m_b_delay_delete;

  /// @brief Track if object was previously notified of deletion by PARAGRAPH
  /// but is kept alive due to pending consumption requests.
  bool                                                   m_b_orphaned;

  /// @brief True if the corresponding PARAGRAPH was created with persistency
  /// enabled.
  bool                                                   m_b_tg_persistent;

  /// @brief The number local producers that have consumer counts that were
  /// deferred and have not yet been subsequently specified.
  size_t                                                 m_deferred_specs;

  /// @brief The aggregate count of unregistered consumers for all producer
  /// tasks. Note, if m_deferred_specs > 0, this count may be negative.
  int                                                    m_unknown_consumer_cnt;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if any producer task executing on this location
  /// has (or possibly has) any outstanding consumers to be registered.
  //////////////////////////////////////////////////////////////////////
  bool has_pending_consumers() const
  {
    return m_unknown_consumer_cnt != 0 || m_deferred_specs != 0;
  }

  using detail::edge_storage::empty;

  //////////////////////////////////////////////////////////////////////
  /// @brief Convert an iterator to @p edge_entry_base to a reference to
  /// the derived class of the entry, edge_entry<T>.
  ///
  /// @tparam T The edge value type.  Parameter is explicitly specified by the
  ///           calling edge_container method.
  ///
  /// @param iter An iterator to @p edge_entry_base.  Must be valid.
  ///
  /// @p down_cast is used to check the downcast with a dynamic_cast when in
  /// debug mode.
  ///
  /// @sa down_cast
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  detail::edge_entry<T>&
  get_entry_ref(edge_entry_t& entry) const
  {
    return down_cast<detail::edge_entry<T>>(entry);
  }

private:
  /////////////////////////////////////////////////////////////////////
  /// @brief Return a reference to the edge_entry<T> associated with task
  /// identifier @p tid.
  ///
  /// If an entry didn't previously exist or was created for signal consumption
  /// (i.e., direct instantiation of @p task_entry_base), construct and return
  /// a new edge_entry<T> object that is properly initialized.
  ///
  /// @tparam T The edge value type.  Parameter is explicitly specified by the
  ///           calling edge_container method.
  ///
  /// @p down_cast is used to check the downcast with a dynamic_cast when in
  /// debug mode.
  ///
  /// @sa down_cast
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  detail::edge_entry<T>&
  lookup_or_insert(index_type tid)
  {
    using derived_edge_entry_t = detail::edge_entry<T>;

    // See if there's an existing entry, if not create one.
    edge_entry_t& base_entry_ref =
      detail::edge_storage::lookup_or_insert(
        tid,
        [this](index_type tid)->edge_entry_t&
          { return *(new derived_edge_entry_t(tid, this->is_persistent())); }
      );

    // This entry is a fully baked (i.e., value holding) entry, created just
    // above or previusly, not some wimpy "signal only" base entry.
    // Downcast like the boss you are and return.
    if (!base_entry_ref.is_signal_only())
      return down_cast<derived_edge_entry_t>(base_entry_ref);

    // This was previously a signal consuming edge_entry_base entry.  Make the
    // entry a full T aware edge_entry<T>, delete the old entry and return.
    derived_edge_entry_t& derived_entry_ref =
      *new derived_edge_entry_t(std::move(base_entry_ref));

    delete &base_entry_ref;

    return derived_entry_ref;
  }


  /////////////////////////////////////////////////////////////////////
  /// @brief Select either the cached @ref edge_entry pointer passed
  /// with the data flow or force a lookup if migration is enabled.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  detail::edge_entry<T>*
  resolve_entry_pointer_on_flow(index_type tid,
                                detail::edge_entry<T>* flow_entry_ptr)
  {
    // It is possible for a receive_value() RMI to arrive at a location after
    // the eviction of the entry that requested the value originally (via remote
    // notifier).  If a successor task depending on this task/value on another
    // location received the value, was released to execution, and subsequently
    // migrated here and all this occurred prior to this receive_value() RMI
    // arriving (there's no ordering guarantee), then the consumers on this
    // location would already have been fired.
    //
    // Additionally, there are cases where the edge_entry* received by RMIs to
    // this method from the producer location is invalid with migration enabled.
    // Consider the above case expanded to include a third consuming task which
    // is migrated to this location after the prior two have finished execution
    // and the corresponding edge_entry is deleted.  It will create a new entry,
    // and for cases where the original tasks' remote flow from the producer
    // location arrives after this point, the entry pointer refers to a
    // reclaimed entry. Hence it is stale and cannot be used to avoid lookup.
    //
    // NOTE - this does point to redundant data being transmitted, but may not
    // be avoidable, unless migration pattern is well known.
    //
    if (!this->migration_enabled())
    {
      stapl_assert(&(this->lookup_expect(tid)) == flow_entry_ptr,
                   "invalid flowed entry pointer detected");

      return flow_entry_ptr;
    }
    else
    {
#if 0
      stapl_warning(0, "found no edge_entry, migration is enabled");
#endif
      return down_cast<detail::edge_entry<T>*>(this->lookup_ptr(tid));
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Reduce the number of local unknown consumers for this object.
  //////////////////////////////////////////////////////////////////////
  void decrement_unknown_count(void)
  {
    --m_unknown_consumer_cnt;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Notify the producer location for the task associated with
  /// @p edge_entry_base iterator @p entry_iter to decrement the unknown
  /// consumer count.  Use the directory to forward the request if the producer
  /// is not local.
  ///
  /// This location has already requested consumption before, in a manner
  /// necessary to cover this consumer (e.g., FULL when this is just a SIGNAL,
  /// etc).
  //////////////////////////////////////////////////////////////////////
  void decrement_unknown_consumers(edge_entry_t& entry)
  {
    if (entry.local_producer_initialized())
    {
      this->decrement_unknown_impl(entry);

      return;
    }

    // else, ship off to the producer loc...
    this->loc_directory().invoke_where(
      std::bind(
        [](p_object& d, index_type producer_tid, location_type location)
        {
          down_cast<edge_container&>(d).decrement_unknown_consumers_impl(
            producer_tid, location);
        },
        std::placeholders::_1, std::placeholders::_2, this->get_location_id()),
      entry.tid());
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Target of directory forwarded request from
  /// @p decrement_unknown_consumers when the producer is not on the consumer
  /// location.
  ///
  /// @sa decrement_unknown_consumers
  ///
  /// @param tid The task identifier of the producer task.
  /// @param loc The location of the consumer associated with the request.
  //////////////////////////////////////////////////////////////////////
  void decrement_unknown_consumers_impl(index_type tid, size_t loc)
  {
    stapl_assert(contains_producer_tid(tid),
      "directory forwarded request for a producer tid not on this location!");

    // stapl_warning(stapl::get_location_id() != loc,
    //   "possible suboptimal order of task creation detected");
    //
    this->decrement_unknown_impl(this->lookup_expect(tid));
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Common implementation of decrement_unknown to avoid code
  /// duplication on local and directory forwarded code paths.  Decrement
  /// local aggregate unknown count and the producer specific count in the
  /// edge entry.
  ///
  /// @param entry_iter Iterator to entry for a local producer whose unknown
  /// count should be decremented.
  //////////////////////////////////////////////////////////////////////
  void decrement_unknown_impl(edge_entry_t& entry)
  {
    entry.decrement_unknown_consumers();

    this->decrement_unknown_count();

    // If producer has already executed, there are no active local users of the
    // value and this is the last unknown consumer, then the value is dead.
    this->try_base_eviction(entry);

    this->try_orphaned_delete();
  }


public:
  using directory_t::get_rmi_handle;

  //////////////////////////////////////////////////////////////////////
  /// @brief Called by successors to create an edge in the @p edge_container.
  ///
  /// Searches the local value cache for information about this
  /// producer_tid before sending a directory forwarded message to the producer.
  ///
  /// @tparam T Edge value type.  Explicitly specified by @p edge_view.
  ///
  /// @param producer_tid The task identifier of the task to set up the
  ///   data flow from.
  /// @param notifier The notifier that should be invoked locally when the
  ///   the data flow is triggered.
  /// @param filter A filter to be applied to the consumed value prior to
  ///   flowing it to the consuming task.
  /// @param b_inc_local_usr_cnt Denotes whether the local reference count
  ///   associated with this version (as defined by @p filter) of the value
  ///   should incremented.
  ///
  /// @p b_inc_local_usr_cnt is generally true, except for a notifier for
  /// the external PARAGRAPH result, which copy the value on notification and
  /// then have no further need to reference the copy in the @p edge_container.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename Filter>
  tuple<detail::edge_entry_base*,
        detail::edge_version_storage<
          typename df_stored_type<
            typename boost::result_of<
              Filter(typename df_stored_type<T>::type)>::type
          >::type>*>
  setup_flow(index_type producer_tid,
             boost::function<
               void (executor_base&,
                     typename df_stored_type<
                       typename boost::result_of<
                         Filter(typename df_stored_type<T>::type)>::type
                     >::type const&)
             > const& notifier,
             Filter const& filter,
             bool b_inc_local_usr_cnt = true);


  //////////////////////////////////////////////////////////////////////
  /// @brief Target of RMI from child PARAGRAPH consuming a value from
  /// a parent PARAGRAPH's input port (i.e., an inter-PARAGRAPH dataflow
  /// instance of @ref edge_container).  Calls @ref setup_flow with
  /// identify filter and without incrementing the local user count.
  /// @ref setup_flow receives a type erased boost::function (which can't be
  /// serialized), this version receives the fully typed notifier which is
  /// type erased prior to calling the former.
  ///
  /// @tparam T Edge value type.  Explicitly specified by @p edge_view.
  /// @param producer_tid The task identifier of the task to set up the
  ///   data flow from.
  /// @param notifier The notifier that should be invoked locally when
  ///   the data flow is triggered.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename Notifier>
  void setup_flow_remote(index_type producer_tid, Notifier notifier)
  {
    using notifier_t =
      boost::function<
        void (executor_base&, typename df_stored_type<T>::type const&)>;

    this->setup_flow<T>(producer_tid,
                        notifier_t(std::move(notifier)),
                        detail::df_identity<T>(), false);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Called by out of gang successors to initiate data flow
  ///   which is handled via a @ref promise.
  ///
  /// @tparam T Edge value type. Explicitly specified by caller.
  ///
  /// @param producer_tid The task identifier of the task to set up the
  ///   data flow from.
  /// @param pr Promise that is fulfilled with the value produced by the
  ///   predecessor.
  ///
  /// @todo Extend to support filtered data flow.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void setup_promise_flow(index_type producer_tid,
                          promise<typename df_stored_type<T>::type> pr);


  //////////////////////////////////////////////////////////////////////
  /// @brief Called by successors to create an signal edge in the
  /// @p edge_container.
  ///
  /// Searches the local value cache for information about this
  /// producer_tid before sending a directory forwarded message to the producer.
  ///
  /// @param producer_tid The task identifier of the task to set up the
  ///   data flow from.
  /// @param notifier The notifier that should be invoked locally when the
  ///   the data flow is triggered.
  //////////////////////////////////////////////////////////////////////
  void setup_signal_flow(index_type producer_tid,
                         signal_notifier_t const& notifier);


  //////////////////////////////////////////////////////////////////////
  /// @brief Mark this location as the execution location for a given task.
  ///
  /// @tparam T Edge value type.  Explicitly specified by @p edge_view.
  ///
  /// @param tid The task identifier of the producer task.
  /// @param consumer_cnt The out degree of the producer task.
  /// @param b_migrated True if this is the insertion of migrated task.  If
  ///   so the migration process updates the producer's execution location and
  ///   there's no need to reregister.
  /// @return A reference to the created @ref edge_entry.  Will be passed to
  ///   the created task.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  detail::edge_entry<T>&
  add_producer(index_type tid, size_t consumer_cnt, bool b_migrated = false)
  {
    // tg::add_task() guards calling this method iff num_succs > 0.
    // tasks have a has_succs bit they pass to tg::processed().
    // tg::processed inspect this and call ec::set_element() iff true.
    stapl_assert(consumer_cnt > 0,
      "edge_container::add_producer encountered consumer <= 0");

    // initialize unknowns with the number of consumers
    // Note: boomeranged requests are not dropped anymore.
    //
    detail::edge_entry<T>& entry_ref = lookup_or_insert<T>(tid);

    entry_ref.set_as_producer(consumer_cnt);

    // Increment aggregrate unknown consumer count kept at container level.
    // If deferred specification of succ_cnt, increment that global count.
    //
    if (consumer_cnt == defer_spec)
      ++m_deferred_specs;
    else
      m_unknown_consumer_cnt += consumer_cnt;

    // Setup of producer entry in value cache complete,
    // notify directory of where this task lives
    //
    if (!b_migrated)
      this->loc_directory().register_key(tid);

    return entry_ref;
  }


private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Add remote notifier for signal consumption on the location
  /// where the producer task is located.  Facilitates inter-location
  /// data flow which will subsequently trigger all associated local
  /// notifications.
  ///
  /// @param producer_tid Task identifier of the producer task to register
  ///   notification request with.
  ///
  /// @param b_is_upgrade True if this notifier upgrades a previous request
  ///   from the same location (i.e., no consumption --> signal).
  ///
  /// @param add_local True if this notifier should be added locally and not
  ///   forwarded to a remote producer through the directory.
  ///
  /// The @p add_local parameter was added with migration support, which
  /// inserts a remote notifier even on the location where the producer task
  /// is (currently) located.  This simplifies the data flow protocol if the
  /// task is later migrated, meaning this location becomes a remote consumption
  /// location.
  //////////////////////////////////////////////////////////////////////
  void send_remote_signal_notifier(const index_type producer_tid,
                                   const bool b_is_upgrade,
                                   const bool add_local = false)
  {
    stapl_assert(!b_is_upgrade, "attempting to upgrade a SIGNAL");

    if (add_local)
    {
      this->add_remote_signal_notifier(producer_tid, this->get_location_id());

      return;
    }

    // else
    this->loc_directory().invoke_where(
      std::bind(
        [](p_object& d, index_type producer_tid, location_type location)
        {
          down_cast<edge_container&>(d).add_remote_signal_notifier(
            producer_tid, location);
        },
        std::placeholders::_1, std::placeholders::_2, this->get_location_id()),
      producer_tid);
  }


public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Add remote notifier for value consumption on the location
  /// where the producer task is located.  Facilitates inter-location
  /// data flow which will subsequently trigger all associated local
  /// notifications.
  ///
  /// @tparam T Edge value type.  Explicitly specified by caller.
  ///
  /// @param producer_tid Task identifier of the producer task to register
  ///   notification request with.
  ///
  /// @param filter Filter to be applied on the producer task value prior to
  ///   consumption by the successor task.
  ///
  /// @param version_id A version identifier created in the @p edge_entry on the
  ///   consuming location associated with @p filter.
  ///
  /// @param b_is_upgrade True if a this notifier upgrades a previous request
  ///   from the same location (i.e., signal --> full).
  ///
  /// @param add_local True if this notifier should be added locally and not
  ///   forwarded to a remote producer through the directory.
  ///
  /// The @p add_local parameter was added with migration support, which
  /// inserts a remote notifier even on the location where the producer task
  /// is (currently) located.  This simplifies the data flow protocol if the
  /// task is later migrated, meaning this location becomes a remote consumption
  /// location.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename Filter>
  void send_remote_notifier(const index_type producer_tid,
                            detail::edge_entry_wrapper<T> entry,
                            Filter const& filter,
                            const unsigned int version_id,
                            const bool b_is_upgrade,
                            const bool add_local = false);


  //////////////////////////////////////////////////////////////////////
  /// @brief Called by the PARAGRAPH to migrate all of an edge_entry's metadata
  /// about a local producer task (i.e., notifications, unknown consumer
  /// counts) to a new location.
  ///
  /// @tparam T Edge value type.  Explicitly specified by caller.
  ///
  /// @param producer_tid Task identifier of producer task to be migrated.
  /// @param dest The new execution location for the task.
  /// @param f The PARAGRAPH method to call on the destination to complete
  ///   the migration process.  (The actual migration is facilitated by the
  ///   edge container since it maintains the directory).
  ///
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename Functor>
  void migrate_entry(const index_type producer_tid,
                     const size_t dest,
                     Functor const& f)
  {
    edge_entry_t& entry = this->lookup_expect(producer_tid);

    using derived_entry_t = detail::edge_entry<T>;

    using remote_notifier_list_t =
      typename derived_entry_t::remote_notifier_list_t;

    using remote_notifier_entry_t =
      typename derived_entry_t::remote_notifier_entry_t;

    remote_notifier_list_t remote_notifications;

    const size_t total_consumer_cnt =
      get_entry_ref<T>(entry).migrate(
        remote_notifications, m_unknown_consumer_cnt);

    // If there are no local consumers, the entry should now be evictable.
    this->try_eviction<T>(entry);

    // prepare setup functor to be executed on the destination,
    // and migrate the entry through the directory.
    this->loc_directory().migrate(
      producer_tid, dest, std::bind(f, total_consumer_cnt));

    using stored_value_t = typename df_stored_type<T>::type;

    while (!remote_notifications.empty())
    {
      remote_notifier_entry_t& notifier_entry = remote_notifications.front();

      remote_notifications.pop_front();

      notifier_entry(*this, producer_tid,
                     detail::edge_version_storage<stored_value_t>(),
                     true, false);

      delete &notifier_entry;
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Called during the migration of a producer task to new location for
  ///   each of its predecessor data flows.
  ///
  /// @tparam T Edge value type.  Explicitly specified by caller.
  ///
  /// @param producer_tid The task identifier of the task that produced this
  ///   value.
  /// @param val The value to be inserted in the @p edge_entry.
  ///
  /// A copy of all predecessor values in the edge container that a migrated
  /// task depends on (flow already set since only runnable tasks are
  /// migratable) are copied to the next execution location to be available
  /// during the task's execution. Create a new edge_entry, unless one already
  /// exists and insert the value.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  tuple<detail::edge_entry_base*,
        detail::edge_version_storage<typename df_stored_type<T>::type>*>
  set_migrated_value(const index_type producer_tid,
                     typename df_stored_type<T>::type const& val)
  {
    using return_t =
      tuple<detail::edge_entry_base*,
            detail::edge_version_storage<typename df_stored_type<T>::type>*>;

    detail::edge_entry<T>& entry_ref = lookup_or_insert<T>(producer_tid);

    if (entry_ref.full_value_set())
    {
      // Enable this when all proxies have ==
      //
      // stapl_assert(entry_ref.value(req) == val,
      //   "value previously set does not equal new value");
    }
    else
      entry_ref.template
        set_value<detail::df_identity<typename df_stored_type<T>::type>>(
          val, *this, 0, false, true);

    entry_ref.increment_local_consumers();

    return return_t(&entry_ref, &entry_ref.get_full_version_storage());
  }


  bool migration_enabled(void) const;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor of edge_container.
  ///
  /// The directory is passed a functor wrapping a method of this class, which
  /// returns true if the given task identifier is registered as a producer on
  /// this location.  Also, initialized such that communication from this object
  /// is not buffered by ARMI and is not included in bookkeeping for fences.
  //////////////////////////////////////////////////////////////////////
  edge_container(bool b_migration_enabled,
                 paragraph_impl::task_graph* tg,
                 bool b_is_persistent)
    : directory_t([this](index_type tid) { return contains_producer_tid(tid); },
                  detail::default_key_mapper<size_t>(),
                  b_migration_enabled,
                  false), // disable buffering on directory messages
      m_tg_ptr(tg),
      m_b_delay_delete(false),
      m_b_orphaned(false),
      m_b_tg_persistent(b_is_persistent),
      m_deferred_specs(0),
      m_unknown_consumer_cnt(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Alternate constructor signature used when the @ref paragraph
  /// defines a custom task to location mapper to be used in the directory.
  //////////////////////////////////////////////////////////////////////
  template<typename TaskMapperParam>
  edge_container(bool b_migration_enabled,
                 paragraph_impl::task_graph* tg,
                 bool b_is_persistent,
                 TaskMapperParam&& task_mapper_param)
    : directory_t([this](index_type tid) { return contains_producer_tid(tid); },
                  std::forward<TaskMapperParam>(task_mapper_param),
                  b_migration_enabled,
                  false), // disable buffering on directory messages
      m_tg_ptr(tg),
      m_b_delay_delete(false),
      m_b_orphaned(false),
      m_b_tg_persistent(b_is_persistent),
      m_deferred_specs(0),
      m_unknown_consumer_cnt(0)
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Destructor of edge_container.
  ///
  /// Verify the the edge_container is empty if not in persistent mode.
  /// If in persistent mode, manually deconstruct all edge entries and then
  /// clear the value cache.
  //////////////////////////////////////////////////////////////////////
  ~edge_container() final;

private:
  edge_container& operator=(edge_container const&) = delete;
  edge_container(edge_container const&)            = delete;

  //////////////////////////////////////////////////////////////////////
  /// @brief Target of RMI via directory request initiated in
  /// @p set_num_succs. Define the number of successors for a task
  /// (@p producer_tid) that was previously delayed by passing
  /// @ref defer_spec to @ref add_producer.
  //////////////////////////////////////////////////////////////////////
  void set_num_succs_local(const index_type producer_tid, size_t num_succs)
  {
    stapl_assert(contains_producer_tid(producer_tid),
      "directory forwarded request for a producer tid not on this location!");

    edge_entry_t& entry = this->lookup_expect(producer_tid);

    entry.delayed_set_num_consumers(num_succs);

    // If num_succs is 0, then we have an interesting case: An edge entry was
    // created when it usually would have been guarded against in the
    // PARAGRAPH. The producer task will call set_element(), and there is no
    // ordering Therefore, if num_succs is 0 then we can ony evict the entry if
    // set_element() has not been called (or if relax the assertions and allow
    // the edge_container to drop the update on the ground.
    if (num_succs || entry.full_value_set())
    {
      this->try_base_eviction(entry);
    }

    stapl_assert(m_deferred_specs > 0,
      "set_num_succs_local received when m_deferred_specs > 0");

    // Fix the aggregate consumer count for the edge_container.
    --m_deferred_specs;

    m_unknown_consumer_cnt += num_succs;
  }

  void pulse_terminator(void) const;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Attempt the eviction (deletion) of an element of the value cache
  /// through the virtual method of the @ref edge_entry_base.
  ///
  /// @param entry Reference to an @ref edge_entry_base object to check
  ///   for evictability.
  ///
  /// Note destruction is runtime polymorphic as well.
  ///
  /// When the concrete type of the entry is known (i.e., the edge value type
  /// is available in the calling context), use @ref try_eviction to avoid
  /// virtual function call.
  ///
  /// @sa try_eviction.
  //////////////////////////////////////////////////////////////////////
  void try_base_eviction(edge_entry_t& entry) const
  {
    if (entry.evictable())
    {
      this->erase(entry);

      if (this->empty())
        this->pulse_terminator();
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Add notifier that will associate the data flow value on this
  ///   location of the outgoing edge of the PARAGRAPH with a the specified
  ///   task's produced value.
  ///
  /// @tparam T Edge value type.  Explicitly specified by caller.
  ///
  /// @param tid The identifier of the task
  ///
  /// @param notifier The result notifier to register with the @ref edge_entry
  ///   for @p tid.
  ///
  /// @return Pointers to associated edge entry and version storage so that
  /// attempt to steal (move) this entry can be made. This occurs if internal
  /// tasks are done with it and only a single location out-flows this value.
  ///
  /// Redirects request to @p edge_container::setup_flow.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename Notifier>
  tuple<detail::edge_entry_base*,
        detail::edge_version_storage<
          typename df_stored_type<
            typename boost::result_of<
              detail::df_identity<T>(typename df_stored_type<T>::type)>::type
          >::type>*>
  set_result(index_type tid, Notifier notifier)
  {
    return this->setup_flow<T>(
      tid, std::move(notifier), detail::df_identity<T>(), false);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Invoked by consumer locations via the directory to add a remote
  ///   notifier to the @ref edge_entry at this producer location.
  ///
  /// @tparam T Edge value type.  Explicitly specified by caller.
  ///
  /// @param tid Task identifier of local producer to register @p notifier
  ///   with.
  ///
  /// @param loc Location where notification request originated from.
  ///
  /// @param notifier Cross location notifier that will apply any filter
  ///   and manage data flow across ARMI.
  ///
  /// @param b_is_upgrade True if a this notifier upgrades a previous request
  ///   from the same location (i.e., signal --> full).
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename Notifier>
  void add_remote_notifier(const index_type tid,
                           const size_t loc,
                           Notifier const& notifier,
                           const bool b_is_upgrade)
  {
    stapl_assert(contains_producer_tid(tid),
      "directory forwarded request for a producer tid not on this location!");

    // We've "boomeranged" back from the producer tid manager back to the
    // consumer loc where we started.  This happens when, on a location,
    // consumer tasks are created before the producer task.
    // This add_remote_notifier request should be downgraded to a
    // decrement_unknown_consumers request.
    //
    if (!this->migration_enabled() && this->get_location_id() == loc)
    {
      this->decrement_unknown_consumers_impl(tid, loc);
      return;
    }

    detail::edge_entry<T>& entry = get_entry_ref<T>(this->lookup_expect(tid));

    entry.add_remote_notifier(*this, notifier, loc, b_is_upgrade);

    this->decrement_unknown_count();

    // If producer has already executed, there are no active local users of the
    // value and this is the last unknown consumer, then the value is dead.
    // and can be safely removed from the value cache.
    //
    this->try_eviction<T>(entry);

    this->try_orphaned_delete();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Invoked by consumer locations via the directory to add a remote
  ///   signal notifier to the edge_entry at this producer location.
  ///
  /// @param tid Task identifier of local producer to register @p notifier
  ///   with.
  ///
  /// @param loc Location where notification request originated from.
  //////////////////////////////////////////////////////////////////////

  void add_remote_signal_notifier(index_type tid, // tid of a local producer
                                  size_t loc)     // source loc of request
  {
    stapl_assert(contains_producer_tid(tid),
      "directory forwarded request for a producer tid not on this location!");

    edge_entry_t& entry = this->lookup_expect(tid);

    entry.add_remote_signal_notifier(*this, loc);

    this->decrement_unknown_count();

    // If producer has already executed, there are no active local users of the
    // value and this is the last unknown consumer, then the value is dead.
    // and can be safely removed from the value cache.
    //
    this->try_base_eviction(entry);

    this->try_orphaned_delete();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Called by @p task_graph::processed (which is invoked when a task
  /// finishes execution) to trigger the initial data flow of producer task.
  ///
  /// @tparam T Edge value type.  Explicitly specified by caller.
  ///
  /// @param tid The task identifier of the task producing the edge value.
  ///
  /// @param val The value produced by the task which will passed (possibly
  ///   after filtering) along its outgoing edge to consumers.
  ///
  /// @p set_value is in turn called on the @p edge_entry and all notifications
  /// for consumer data flow are invoked.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename ValueParam>
  void set_element(const index_type tid, ValueParam&& val)
  {
    edge_entry_t& entry = this->lookup_expect(tid);

    // NOTE - below conditions means the initialization of this section of
    // PARAGRAPH hasn't finished, sign of possible slowdown over optimal,
    // suggests one should maybe increase task creation window.
#if 0
    if (entry.m_unknown_consumer_cnt > 0)
      std::cout << get_location_id() << ": set_element(" << tid
                << ") found unknown consumer\n";
#endif

    // This method is only called on producer location by tg::processed;
    // Hence, always a FULL request type.
    get_entry_ref<T>(entry).template
      set_value<detail::df_identity<typename df_stored_type<T>::type>>(
        std::forward<ValueParam>(val), *this, 0, true);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Called by @p task_graph::processed (which is invoked when a task
  /// finishes execution) to trigger the initial data flow of producer task when
  /// the task's return value type is void.
  ///
  /// @param tid The task identifier of the task producing the edge value.
  //////////////////////////////////////////////////////////////////////
  void set_element(index_type tid)
  {
    edge_entry_t& entry = this->lookup_expect(tid);

    // NOTE - below conditions means the initialization of this section of
    // PARAGRAPH hasn't finished, sign of possible slowdown over optimal,
    // suggests one should maybe increase task creation window.
#if 0
    if (entry.m_unknown_consumer_cnt > 0)
    {
      std::cout << get_location_id() << ": set_element(" << tid
                << ") found unknown consumer\n";
    }
#endif

    // This method is only called on producer location by tg::processed;
    // Hence, always a FULL request type.
    entry.set_value(*this);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called by a producer sibling nested @ref paragraph to flow
  /// values into the input port this instance of an @ref edge_container
  /// represents.  If this is first time this is called for this task id,
  /// (true in all cases for ephemeral pgs), call @ref add_producer prior
  /// to redirecting to the basic @ref set_element.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename ValueParam>
  void nested_sibling_set_element(const index_type tid, ValueParam&& val);

  //////////////////////////////////////////////////////////////////////
  /// @brief Called by the PARAGRAPH to request invocation of the
  /// notifier represented by @p notifier_ptr when the task identified by
  /// @p producer_tid has finished execution.
  ///
  /// Wraps the notifier appropriately based on whether persistency is
  /// enabled or not and then redirects to @p edge_container::setup_signal_flow.
  //////////////////////////////////////////////////////////////////////
  void setup_signal(index_type producer_tid,
                    edge_container::local_notifier_t* notifier_ptr)
  {
    using base_local_notifier_t = edge_container::local_notifier_t;
    using ephemeral_ptr_t       = base_local_notifier_t*;
    using persistent_ptr_t      = boost::intrusive_ptr<base_local_notifier_t>;

    this->setup_signal_flow(
      producer_tid,
      this->is_persistent() ?
        signal_notifier_t(bind(&base_local_notifier_t::operator(),
               persistent_ptr_t(notifier_ptr), _1))
        : signal_notifier_t(bind(&base_local_notifier_t::operator(),
               ephemeral_ptr_t(notifier_ptr),  _1)));
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Target of RMI invoked by a remote signal notifier to trigger
  ///   data flow on the remote consumption location.
  ///
  /// @param tid The task identifier this whose signal notification this
  ///   invocation represents.
  ///
  /// Redirect to edge_entry for @p tid so that it can notify all local signal
  /// consumers.
  //////////////////////////////////////////////////////////////////////
  void receive_signal(const index_type tid);


  //////////////////////////////////////////////////////////////////////
  /// @brief Target of RMI invoked by a remote notifier to trigger
  ///   data flow on the remote consumption location.
  ///
  /// @tparam T Edge value type.  Explicitly specified by caller.
  ///
  /// @param tid The task identifier this whose signal notification this
  ///   invocation represents.
  ///
  /// @param entry Trivial iterator to the @ref edge_entry for the given
  /// value on this location.
  ///
  /// @param val The value associated with this data flow.
  ///
  /// @param version_id The version identifier associated with this value.
  ///   Associates this data flow with the filter that was originally specified
  ///   when the edge(s) was created.
  ///
  /// Redirect to edge_entry for @p tid so that it can notify all local
  /// associated consumers of this version (and any dependent versions).
  ///
  /// @todo tid parameter is only used in assertions and migration supporting
  /// portions of the protocol.  Guard as appropriate to avoid needlessly
  /// sending these bits.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename Filter>
  void receive_value(const index_type tid,
                     detail::edge_entry_wrapper<T> entry,
                     typename df_stored_type<
                       typename boost::result_of<
                         Filter(typename df_stored_type<T>::type)
                       >::type
                     >::type const& val,
                     const unsigned int version_id);


  //////////////////////////////////////////////////////////////////////
  /// @brief Target of RMI invoked by a remote notifier to trigger
  ///   data flow on the remote consumption location when moving the
  ///   flowed value.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename Filter>
  void receive_move_value(const index_type tid,
                          detail::edge_entry_wrapper<T> entry,
                          typename df_stored_type<
                            typename boost::result_of<
                              Filter(typename df_stored_type<T>::type)
                            >::type
                          >::type&& val,
                          const unsigned int version_id);


  //////////////////////////////////////////////////////////////////////
  /// @brief Target of RMI invoked by a remote notifier to trigger
  ///   data flow on the remote consumption location when sharing the
  ///   flowed value.
  //////////////////////////////////////////////////////////////////////
  template<typename T, typename Filter>
  void receive_shared_value(const index_type tid,
                            detail::edge_entry_wrapper<T> entry,
                            immutable_shared<
                              typename df_stored_type<T>::type> const& wrapper,
                            const unsigned int version_id);


  //////////////////////////////////////////////////////////////////////
  /// @brief Attempt the eviction (deletion) of an element of the value cache
  ///  when the concrete type (i.e., edge value type) is known at the call
  ///  site.
  ///
  /// @tparam T Edge value type.  Explicitly specified by caller.
  ///
  /// @param entry A reference to a @ref edge_entry_base object to check
  ///   for evictability.
  ///
  /// @sa try_base_eviction.
  ///
  /// Right now we have an aggressive eviction policy on locations not producing
  /// a given value. Once all currently initialized consumers execute, the value
  /// is evicted, regardless of the size of the cache.  Being more lazy might
  /// allow consumers that later initialize to reuse the value instead of being
  /// forced to wait for a refetch.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void try_eviction(edge_entry_t& entry) const
  {
    if (get_entry_ref<T>(entry).evictable())
    {
      this->erase(entry);

      if (this->empty())
        this->pulse_terminator();
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Attempt the eviction (deletion) of an element of the value cache
  ///  when the concrete type (i.e., edge value type) is known at the call
  ///  site.
  ///
  /// @tparam T Edge value type.  Explicitly specified by caller.
  ///
  /// @param entry A reference to a @ref detail::edge_entry object to check
  ///   for evictability.
  ///
  /// This version is used when reference to derived class is already present.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void try_eviction(detail::edge_entry<T>& entry) const
  {
    if (entry.evictable())
    {
      this->erase(entry);

      if (this->empty())
        this->pulse_terminator();
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Resets edge entry data flow triggers and all notifiers
  ///   to prepare for a reinvocation of a persistent PARAGRAPH.
  ///
  /// Called by @ref task_graph::reset.
  //////////////////////////////////////////////////////////////////////
  void reset_entry_values(void) const
  {
    this->for_each([](edge_entry_t& entry) { entry.reset_values(); });
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Notify the edge_container that it is no longer needed by the
  /// PARAGRAPH. Termination detection (at least on the location) has
  /// succeeded.
  ///
  /// It is possible that the edge_container made need to stay alive longer
  /// if there are pending notifications stil being flushed.
  ///
  /// @todo verify that deletion can in fact still be delayed in the
  /// current implementation.
  //////////////////////////////////////////////////////////////////////
  void release(void)
  {
    stapl_assert(!has_pending_consumers(), "release found pending consumers");

    if (!m_b_delay_delete)
    {
      delete this;
      return;
    }

    // else
    m_b_orphaned = true;
  }


  task_graph& tg() const
  {
    return *m_tg_ptr;
  }


  directory_t& loc_directory(void) // const
  {
    return static_cast<directory_t&>(*this);
  }


  bool is_persistent(void) const
  {
    return m_b_tg_persistent;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns true if this location is currently marked as the execution
  /// location for the given task.
  //////////////////////////////////////////////////////////////////////
  bool contains_producer_tid(index_type tid) const
  {
    edge_entry_t* entry_ptr = this->lookup_ptr(tid);

    return entry_ptr != nullptr && entry_ptr->local_producer_initialized();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Define the number of successors for a task (@p producer_tid) that
  /// was previously delayed by passing @p defer_spec to @ref add_producer.
  //////////////////////////////////////////////////////////////////////
  void set_num_succs(index_type producer_tid, size_t num_succs)
  {
    this->loc_directory().invoke_where(
      std::bind(
        [](p_object& e, index_type tid, size_t n_succs)
          { down_cast<edge_container&>(e).set_num_succs_local(tid, n_succs); },
        std::placeholders::_1, std::placeholders::_2, num_succs),
      producer_tid);
  }


private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Try a deletion in methods that may be called after a deletion has
  ///   been attempted by the PARAGRAPH.  If this indeed the case, delete the
  ///   edge_container unless there are remaining unregistered consumption
  ///   requests that need to be serviced.
  //////////////////////////////////////////////////////////////////////
  void try_orphaned_delete(void)
  {
    if (m_b_orphaned && !has_pending_consumers())
    {
      stapl_assert(!m_b_tg_persistent,
        "unexpected condition in try_orphaned_delete");

      std::cout << get_location_id() << ": try_orphaned_delete succeeded\n";

      delete this;
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Pause any attempt to delete the edge_container while
  ///   @p edge_entry processing in progress.
  //////////////////////////////////////////////////////////////////////
  void start_hold(void)
  {
    m_b_delay_delete = true;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Stops a pause on deletion due to edge_entry processing.  If a
  ///   deletion request was received during hold, service it if there are
  ///   are not remaining unregistered consumption requests for local producers.
  //////////////////////////////////////////////////////////////////////
  void stop_hold(void)
  {
    if (m_b_orphaned && !has_pending_consumers())
    {
      delete this;
      return;
    }

    // else
    m_b_delay_delete = false;
  }
}; // class edge_container

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_EDGE_CONTAINER_H


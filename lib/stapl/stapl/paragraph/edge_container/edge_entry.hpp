/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_EDGE_ENTRY_HPP
#define STAPL_PARAGRAPH_EDGE_ENTRY_HPP

#include <boost/intrusive/list.hpp>
#include <boost/intrusive/unordered_set.hpp>

#include <stapl/paragraph/edge_container/edge_entry_base.hpp>
#include <stapl/paragraph/edge_container/edge_notifier_wrapper.hpp>
#include <stapl/paragraph/edge_container/edge_remote_notifier.hpp>

#include <stapl/paragraph/edge_container/remote_edge_version_entry.hpp>

#include <stapl/utility/down_cast.hpp>
#include <stapl/utility/empty_class.hpp>

#include <ostream>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Wraps notifier passed to edge_entry.  Provides storage and access
/// method for the location associated with the base remote notifier.
///
/// @ingroup pgEdgeEntry
///
/// @tparam Notifier The base remote notifier to be used.
//////////////////////////////////////////////////////////////////////
template<typename Notifier>
class remote_notifier
  : private Notifier,       // inheritance for empty base optimization.
    public location_storage // location this remote notifier refers to.
{
public:
  remote_notifier(Notifier notifier, size_t loc)
    : Notifier(std::move(notifier)),
      location_storage(loc)
  { }

  STAPL_USE_MANAGED_ALLOC(remote_notifier)

  //////////////////////////////////////////////////////////////////////
  /// @brief Wraps function operator of @p Notifier, passing the
  ///        associated location appropriately in the parameter list.
  ///
  /// @param ct The @p edge_container of the PARAGRAPH.
  ///
  /// @param tid Task identifier of the producer task for this edge.
  ///
  /// @param value Produced value to be flowed to the remote consumer location.
  ///
  /// @param b_migrate Whether this invocation represents notification or
  ///                  or notifier migration request.
  ///
  /// @sa edge_notifier_wrapper
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void operator()(edge_container& ct, std::size_t tid,
                  edge_version_storage<T> const& version,
                  bool b_migrate, bool b_moveable) const
  {
    Notifier::operator()(
      ct, this->location(), tid, version, b_migrate, b_moveable);
  }
}; // class remote_notifier


//////////////////////////////////////////////////////////////////////
/// @brief Lazily provides access to the @ref edge_version_storage
/// associated with an edge version entry if desired by another
/// version entry.
///
/// Used to allow access to the full version value by remote version
/// entry list if needed to satisfy notifications.
//////////////////////////////////////////////////////////////////////
template<typename T>
class entry_storage_access
{
private:
  using entry_t =
    edge_version_entry<
      T, boost::function<void (executor_base&, T const&)>,
      boost::intrusive::make_slist_base_hook<>::type>;

  entry_t const* m_entry_ptr;

public:
  entry_storage_access(entry_t const* entry_ptr)
    : m_entry_ptr(entry_ptr)
  { }

  edge_version_storage<T> const&
  operator()(void) const
  {
    stapl_assert(m_entry_ptr != nullptr,
                 "attempting to deferefence null entry pointer");

    return m_entry_ptr->version_storage();
  }
}; // class entry_storage_access


//////////////////////////////////////////////////////////////////////
/// @brief Lazily provides access to the value in an edge version
/// entry's underlying storage if desired by another version entry.
///
/// Used to allow access to the full version value by filtered version
/// entry list if needed to satisfy notifications.
//////////////////////////////////////////////////////////////////////
template<typename T>
class entry_value_access
{
private:
  using entry_t =
    edge_version_entry<
      T, boost::function<void (executor_base&, T const&)>,
      boost::intrusive::make_slist_base_hook<>::type>;

  entry_t const* m_entry_ptr;

public:
  entry_value_access(entry_t const* entry_ptr)
    : m_entry_ptr(entry_ptr)
  { }

  T const& operator()(void) const
  {
    stapl_assert(m_entry_ptr != nullptr,
                 "attempting to deferefence null entry pointer");

    return m_entry_ptr->value();
  }
}; // class entry_value_access


//////////////////////////////////////////////////////////////////////
/// @brief The primary storage class for dependence and data flow values
/// in the PARAGRAPH.  One instance of this class or the base class
/// @ref edge_entry_base exists for each producer task, both on the
/// location where it executes as well as on all locations with active
/// consumers.
///
/// @tparam The return type of the producer task's workfunction.
///
/// @ingroup pgEdgeEntry
///
/// Each entry maintains a collection of notifier lists called versions.
/// A version may be held in the entry object frame (for now remote and signal
/// versions are held this way) or in a versions list held as data member
/// (currently the case for filtered and full versions).  This is strictly an
/// implementation detail that likely warrants further investigation.
///
/// The only component interacting with the edge_entry directly is the
/// @p edge_container which maintains all entries for a given location in a
/// container.  The @p edge_container redirects most requests from outside
/// callers to the appropriate entry methods.
///
/// @todo Boost.Function usage, while generic for notifier storage, may
/// introduce overhead.  Verify the genericity is actually needed or if the
/// notifier types are generally well known and can be used directly with a
/// little code refactoring.
///
/// @todo A smarter, adaptive storage methodology for versions that decided
/// based on needs of the PARAGRAPH (i.e., does it needs signal, filters, etc)
/// whether to store a version in the edge_entry frame or in a list such as
/// m_version could reduce memory usage and increase locality for the common,
/// simple uses of this class template.  For example, edge_entry<T, FULL_ONLY>.
///
/// @todo Consider another derived class of edge_entry_base, specifically for
/// use on the producer location.  Doing so would reduce the memory consumption
/// on consumer only locations (at the expense of additional code and template
/// instantiations.
///
/// @todo The lifetime of all value versions are tied directly to the full
/// entry.  Keeping per version reference counts could allow us to retire
/// individual versions as they are no longer need and (maybe) reduce overall
/// @p edge_container storage.
///
/// @todo Implementing an apply_if_exists(req, F) for the versions list could
/// avoid some unnecessary creations of versions that don't actually need to
/// exist (at least exist yet).
///
/// @todo The lookup of the full entry some contexts (i.e.,
/// @p add_remote_notifier) could be avoided by caching the full flow_set bit
/// in the entry frame and pass a functor other versions to retrieve the full
/// value only if needed.
//////////////////////////////////////////////////////////////////////
template<typename T>
class edge_entry
  : public edge_entry_base
{
private:
  /// @brief the actual stored data type in the entry.
  typedef typename df_stored_type<T>::type          stored_value_t;

  /// @brief The base class shared by full and filtered versions which defines
  /// an entry in the versions list.
  typedef edge_version_entry_base<
    boost::intrusive::make_slist_base_hook<>::type> edge_version_entry_t;

  /// @brief The versions list type which holds full and filtered version
  /// notifier lists.
  typedef boost::intrusive::make_slist<
    edge_version_entry_t,
    boost::intrusive::constant_time_size<false>,
    boost::intrusive::cache_last<true>>::type       edge_version_list_t;

  /// @brief The type of the full version notifier list.
  typedef edge_version_entry<
    stored_value_t,
    boost::function<
      void (executor_base&, stored_value_t const&)>,
    boost::intrusive::make_slist_base_hook<>::type> full_edge_version_entry_t;

  /// @brief The type of the filtered versions notifier list.
  typedef filtered_edge_versions<
    stored_value_t,
    boost::intrusive::make_slist_base_hook<>::type> filtered_edge_versions_t;

  /// @brief Initial wrapper placed around remote notifier to erase concrete
  /// type.
  typedef boost::function<
    void (edge_container&, size_t, std::size_t,
          edge_version_storage<stored_value_t> const&, bool, bool)
  >                                                 base_remote_notifier_t;

  /// @brief The remote notifier stored by this edge_entry.  Wraps all the
  /// @p base_remote_notifier_t and stores the associated remote location.
  typedef remote_notifier<base_remote_notifier_t>   remote_notifier_t;

  /// @brief The remote version list type.
  typedef remote_edge_version_entry<
    stored_value_t, remote_notifier_t, empty_class> remote_edge_version_entry_t;

public:
  /// @brief Remote version entry type reflected publicly for migration support.
  typedef typename remote_edge_version_entry_t::entry_t remote_notifier_entry_t;

/// @brief Remote version list type reflected publicly for migration support.
  typedef typename remote_edge_version_entry_t::notifier_list_t
    remote_notifier_list_t;

private:
  typedef typename edge_version_list_t::const_iterator  versions_citer_t;
  typedef typename edge_version_list_t::iterator        versions_iter_t;

  /// @brief List of notifier lists associated with different versions of the
  /// produced value.  Currently contains at most one full entry and one
  /// filtered entry (which stores a list of all filtered versions).
  edge_version_list_t                                   m_versions;

  /// @brief Stores notifiers for remote consumer locations.
  remote_edge_version_entry_t                           m_remote_version;

public:
  edge_entry& operator=(edge_entry const&) = delete;
  edge_entry(edge_entry const&)            = delete;

  explicit
  edge_entry(std::size_t tid, bool b_persistent)
    : edge_entry_base(tid, b_persistent, false),
      m_remote_version()
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Constructor called to upgrade a edge_entry_base element of the
  /// edge_container to edge value typed instance of this class template.
  ///
  /// This is effectively a move constructor and that calls the similarly
  /// defined move constructor of @ref edge_entry_base.
  //////////////////////////////////////////////////////////////////////
  explicit
  edge_entry(edge_entry_base&& other)
    : edge_entry_base(std::move(other))
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Remove all notifications if not part of persistent PARAGRAPH
  /// and remove version entries stored @p m_versions.
  //////////////////////////////////////////////////////////////////////
  ~edge_entry() final
  {
    if (!this->is_persistent())
      this->cleanup_notifications();

    m_versions.clear_and_dispose(version_entry_disposer());

    stapl_assert(this->persistent_evictable(),
      "~edge_entry(): not evictable");
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Clear all local and remote notifications stored in the entry.
  ///
  /// Either called in the destructor or as part of the delayed edge_container
  /// destruction used when persistency support is enabled.
  //////////////////////////////////////////////////////////////////////
  void cleanup_notifications(void) final
  {
    this->m_signal_version.cleanup_notifications();
    this->m_remote_version.cleanup_notifications();

    for (auto&& version : m_versions)
      version.cleanup_notifications();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Freestanding version of operator defined for the @p edge_entry
  /// in terms of the entries' task identifiers.  Used by the Boost.Intrusive
  /// @p unordered_set in which all entries are stored by the @p edge_container.
  //////////////////////////////////////////////////////////////////////
  friend bool operator==(edge_entry const& lhs, edge_entry const& rhs)
  {
    return lhs.m_tid == rhs.m_tid;
  }

  STAPL_USE_MANAGED_ALLOC(edge_entry)

  //////////////////////////////////////////////////////////////////////
  /// @brief Define insertion operator on ostream. Provides useful debugging
  /// information.
  ///
  /// @param os The output stream to insert into.
  /// @param entry The edge entry to insert information about into the stream.
  //////////////////////////////////////////////////////////////////////
  friend
  std::ostream& operator<<(std::ostream& os, edge_entry<T> const& entry)
  {
    return os << "Cache Entry:\n"
      << "Tid                        = " << entry.m_tid << "\n"
      << "Unknown Consumer Cnt       = " << entry.m_unknown_consumer_cnt << "\n"
      << "Local User Cnt             = " << entry.m_local_user_cnt << "\n"
      << "Empty Local Notifications  = " <<
      entry.empty_local_notifications() << "\n"
      << "Empty Remote Notifications = "
      << entry.m_remote_version.empty_notifications() << "\n"
      << "Flushing Remote            = "
      << entry.m_remote_version.test_flushing() << "\n";
  }


private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if all signal, filter, and full consumer notification
  /// lists are empty.
  ///
  /// @sa edge_container::setup_flow(
  //////////////////////////////////////////////////////////////////////
  bool empty_local_notifications() const
  {
    if (!this->empty_signal_notifications())
      return false;

    for (auto&& version : m_versions)
      if (!version.empty_notifications())
        return false;

    return true;
  }


public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if there are any local notifications in this entry
  /// that have not yet been flushed by a data flow trigger.
  //////////////////////////////////////////////////////////////////////
  bool has_unnotified_local_consumer() const
  {
    return !this->empty_local_notifications();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Receive a local consumption request with associated @p notifier
  ///   and @p filter.  Integrate it into edge_entry's local dependence
  ///   representation and signal caller (@ref edge_container::setup_flow)
  ///   if any remote notification is required.
  ///
  /// @param ct the @p edge_container holding this @p edge_entry.
  /// @param notifier The notifier to invoke when the requested data flow
  ///   is complete and available for consumption.
  /// @param filter The filtering function to be applied on the producer task's
  ///   value prior to passing it successor.
  /// @param b_inc_local_usr_cnt Denotes whether this is a consumer that will
  ///   rely on the entry's copy of the flowed value.  Guides internal reference
  ///   counting.
  /// @return Pair containing add_status variable to guide creation of
  ///  remote notifier and the version identifier associated with filter
  ///  to use if remote notifier is created.
  //////////////////////////////////////////////////////////////////////
  template<typename Filter>
  tuple<df_add_status, unsigned int,
        edge_version_storage<
          typename df_stored_type<
            typename boost::result_of<
              Filter(stored_value_t)>::type>::type>&>
  add_local_notifier(edge_container& ct,
                     boost::function<
                       void (executor_base&,
                             typename df_stored_type<
                               typename boost::result_of<
                                 Filter(typename df_stored_type<T>::type const&)
                               >::type
                             >::type const&)
                     > const& notifier,
                     bool b_inc_local_usr_cnt,
                     Filter const& filter)
  {
    // If filtered or full data is flowed (this isn't just a signal), a proxy
    // to this value will be constructed ((i.e., not copy constructed) either
    // in create_task() or by the return value of the task graph.  Increment the
    // local_user_cnt to hold value (even after event notifier invoked) until
    // this occurs.  Note the destructor of the edge_accessor contains the
    // matching decrement.
    if (b_inc_local_usr_cnt)
      this->increment_local_consumers();

    filtered_edge_versions_t& filtered_versions = get_filtered_versions();

    auto full_iter = this->iterator_lookup_version(FULL);

    full_edge_version_entry_t* const full_ptr =
      full_iter == m_versions.end()
      ?  nullptr : &down_cast<full_edge_version_entry_t&>(*full_iter);

    auto ret_val =
      filtered_versions.add_notifier(
        notifier, filter, ct.tg().executor(),
        entry_value_access<stored_value_t>(full_ptr),
        full_ptr == nullptr ? false : full_ptr->test_flow(),
        ct.tg().migration_enabled(), is_persistent());

    typedef tuple<
      df_add_status, unsigned int,
      edge_version_storage<
        typename df_stored_type<
          typename boost::result_of<
            Filter(stored_value_t)>::type>::type>&> return_t;

    return return_t(
      this->compute_request_status(FILTERED, get<0>(ret_val), get<2>(ret_val)),
      get<1>(ret_val) + 1,
      get<3>(ret_val));
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Signature of function for identity / full value consumption.
  //////////////////////////////////////////////////////////////////////
  tuple<df_add_status, unsigned int,
        edge_version_storage<stored_value_t>&>
  add_local_notifier(edge_container& ct,
                     boost::function<
                       void (executor_base&, stored_value_t const&)
                     > const& notifier,
                     bool b_inc_local_usr_cnt,
                     detail::df_identity<stored_value_t> const&)
  {
    // If filtered or full data is flowed (this isn't just a signal), a proxy
    // to this value will be constructed ((i.e., not copy constructed) either
    // in create_task() or by the return value of the task graph.  Increment the
    // local_user_cnt to hold value (even after event notifier invoked) until
    // this occurs.  Note the destructor of the edge_accessor contains the
    // matching decrement.
    if (b_inc_local_usr_cnt)
      this->increment_local_consumers();

    full_edge_version_entry_t& full_version = get_full_version();

    auto& storage_ref =
      full_version.add_notifier(notifier, ct.tg().executor(), is_persistent());

    return tuple<df_add_status, unsigned int,
                 edge_version_storage<stored_value_t>&>
      (this->compute_request_status(FULL, full_version.test_flow()),
       0, storage_ref);
  }


  edge_version_storage<stored_value_t>&
  get_full_version_storage(void)
  {
    return get_full_version();
  }


  edge_version_storage<stored_value_t> const&
  get_full_version_storage(void) const
  {
    return get_full_version();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Add a notifier to transmit a producer value (full or filtered) to
  /// a remote consumer location and trigger data flow of local consumer
  /// edges there.
  ///
  /// @param ct The edge container this entry is held by.
  /// @param notifier The notifier to called when the producer task finishes
  ///   execution (encapsulate any filter function).
  /// @param loc The location this notifier originated from.
  /// @param b_upgrade Denotes whether this request upgrades a previous remote
  ///   notification from location @p loc to higher request level
  ///   (i.e., FILTERED --> FULL).
  ///
  /// @sa remote_edge_version_entry
  ///
  /// Currently only called on the producer location.
  /// @todo Use is_moveable<T> metafunction from runtime to guard generation of
  ///   move code paths for basic types, etc.
  //////////////////////////////////////////////////////////////////////
  template<typename Notifier>
  void
  add_remote_notifier(edge_container& ct,
                      Notifier const& notifier,
                      const size_t loc,
                      const bool b_upgrade)
  {
    stapl_assert(this->local_producer_initialized(),
      "edge_entry::add_remote_notifier: notifier for uninitialized producer");

    full_edge_version_entry_t const& full_version = get_full_version();

    const bool b_moveable = m_unknown_consumer_cnt==1 && m_local_user_cnt==0;

    m_remote_version.add_notifier(
      remote_notifier_t(notifier, loc), tid(), ct,
      entry_storage_access<stored_value_t>(&full_version),
      full_version.test_flow(),
      is_persistent(), b_upgrade, b_moveable
#ifndef STAPL_NDEBUG
      , m_b_consumers_before_producer
#endif
    );

    // Regardless of whether we can send the value yet or not, we've been
    // notified of a previously unknown consumer.  Decrement unknown count.
    this->decrement_unknown_consumers();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Add a signal notifier to notify a remote consumer location
  /// when a producer task finishes execution.
  ///
  /// @param ct The edge container this entry is held by.
  /// @param loc The location this notifier originated from.
  //////////////////////////////////////////////////////////////////////
  void add_remote_signal_notifier(edge_container& ct, size_t loc) final
  {
    typedef edge_notifier_wrapper<
      T, detail::edge_remote_signal_notifier<stored_value_t>> notifier_t;

    this->add_remote_notifier(ct, notifier_t(), loc, false);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Used to trigger data flow in this entry for edges that can be
  /// serviced with the version of the value that has been passed via
  /// immutable sharing.
  //////////////////////////////////////////////////////////////////////
  void set_shared_value(immutable_shared<stored_value_t> const& wrapper,
                        edge_container& ct, unsigned int version_id,
                        bool b_caller_is_set_element = false,
                        bool b_caller_is_migration   = false)
  {
    stapl_assert(!b_caller_is_migration, "immutable detected, migration");
    stapl_assert(version_id == 0, "immutable, filtering");

    const bool b_migration_enabled = ct.tg().migration_enabled();

    // Using a pointer to handle case when edge_container backs a
    // pg input port initialized prior to executor construction.
    // notifications can't exist (as factory hasn't been called for setup
    // of internal consumers), but value may be set if flow already available.
    auto executor_ptr = ct.tg().executor_ptr();

    // Either filtered or full means signal requests can be notified.
    if (!this->signal_set())
    {
      // Passing a pointer to handle case when edge_container backs a
      // PG input port initialized prior to executor construction.
      // Notifications can't exist (as factory hasn't been called for setup
      // of internal consumers.
      this->set_signal(executor_ptr, b_migration_enabled);
    }
    else
      stapl_assert(this->is_persistent() || this->empty_signal_notifications(),
        "m_b_signal_flow_set with pending SIGNAL notifications");

    constexpr bool b_expect_exist = false;

    auto& full_version_entry = get_full_version(b_expect_exist);

    full_version_entry.set_value(
      wrapper, executor_ptr, b_migration_enabled, ct.is_persistent());

    versions_iter_t filtered_iter = iterator_lookup_version(FILTERED);

    // val is subject to move above and may be emptied now. Use value stored
    // in full_version_entry to service requests below (which are initialized
    // by copy, for now).
    if (b_caller_is_set_element)
    {
      constexpr bool b_moveable = false;

      stapl_assert(!b_moveable || filtered_iter == m_versions.end(),
        "expected empty filtered notifier list");

      m_remote_version.set_value(
        ct, tid(), entry_storage_access<stored_value_t>(&full_version_entry),
        is_persistent(), b_moveable);
    }

    // use set_value_full so filtered_version applies filter before notifying.
    if (filtered_iter != m_versions.end())
    {
      down_cast<filtered_edge_versions_t&>(*filtered_iter).set_value_full(
       full_version_entry.value(), executor_ptr,
       b_migration_enabled, is_persistent());
    }

    if (b_caller_is_set_element)
      ct.try_eviction(*this);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation helper function for @ref set_value to handle
  /// full version set value calls.
  //////////////////////////////////////////////////////////////////////
  template<typename Filter, typename ValueParam>
  void
  set_value_impl(ValueParam&& val, edge_container& ct,
                 unsigned int version_id,
                 bool b_caller_is_set_element, bool b_migration_enabled,
                 bool b_expect_exist, std::true_type)
  {
    stapl_assert(version_id == 0, "Unexpected version_id (expected full)");

    // else, req == FULL, fulfill lower level requests if possible
    auto& full_version_entry = get_full_version(b_expect_exist);

    // Using a pointer to handle case when edge_container backs a
    // PG input port initialized prior to executor construction.
    // Notifications can't exist (as factory hasn't been called for setup
    // of internal consumers), but value may be set if flow already available.
    auto executor_ptr = ct.tg().executor_ptr();

    // If it's a basic type, if there is exactly one consumer or if we already
    // know for certain that there are no remote consumers, store the value in
    // the version entry object frame.  Otherwise, give ARMI the lifetime
    // management responsibilities for the value, in preparation for potential
    // inter-location sharing (costs one heap allocation + reference counting
    // overhead).
    if (is_basic<stored_value_t>::value
        || this->has_single_consumer()
        || (m_unknown_consumer_cnt==0
            && m_remote_version.empty_notifications()))
    {
      full_version_entry.set_value(
        std::forward<ValueParam>(val), executor_ptr,
        b_migration_enabled, is_persistent());
    }
    else
    {
      full_version_entry.set_value(
        make_immutable_shared<stored_value_t>(std::forward<ValueParam>(val)),
        executor_ptr, b_migration_enabled, is_persistent());
    }

    versions_iter_t filtered_iter = iterator_lookup_version(FILTERED);

    // val is subject to move above and may be emptied now. Use value stored
    // in full_version_entry to service requests below (which are initialized
    // by copy, for now).
    if (b_caller_is_set_element)
    {
      const bool b_moveable = m_unknown_consumer_cnt==0 && m_local_user_cnt==0;

      stapl_assert(!b_moveable || filtered_iter == m_versions.end(),
        "expected empty filtered notifier list");

      m_remote_version.set_value(
        ct, tid(), entry_storage_access<stored_value_t>(&full_version_entry),
        is_persistent(), b_moveable);
    }

    // use set_value_full so filtered_version applies filter before notifying.
    if (filtered_iter != m_versions.end())
    {
      down_cast<filtered_edge_versions_t&>(*filtered_iter).set_value_full(
        full_version_entry.value(), executor_ptr,
        b_migration_enabled, is_persistent());
    }

    if (b_caller_is_set_element)
      ct.try_eviction(*this);
 }


  //////////////////////////////////////////////////////////////////////
  /// @brief Implementation helper function for @ref set_value to handle
  /// filtered versions set value calls.
  //////////////////////////////////////////////////////////////////////
  template<typename Filter, typename ValueParam>
  void
  set_value_impl(ValueParam&& val, edge_container& ct,
                 unsigned int version_id,
                 bool b_caller_is_set_element, bool b_migration_enabled,
                 bool b_expect_exist, std::false_type)
  {
    stapl_assert(version_id > 0, "Unexpected version_id (expected filtered)");

    // Using a pointer to handle case when edge_container backs a
    // pg input port initialized prior to executor construction.
    // notifications can't exist (as factory hasn't been called for setup
    // of internal consumers), but value may be set if flow already available.
    auto executor_ptr = ct.tg().executor_ptr();

    get_filtered_versions(b_expect_exist). template set_value<Filter>(
      std::forward<ValueParam>(val), version_id, executor_ptr,
      b_migration_enabled, is_persistent(), b_expect_exist);
  }


  template<typename Filter>
  void set_value(immutable_shared<stored_value_t>&& wrapper,
                 edge_container& ct,
                 unsigned int version_id,
                 bool b_caller_is_set_element = false,
                 bool b_caller_is_migration   = false)
  {
    this->set_shared_value(wrapper, ct, version_id,
                           b_caller_is_set_element,
                           b_caller_is_migration);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Used to trigger data flow in this entry for edges that can be
  /// serviced with the version of the value that has been passed.
  ///
  /// @param val The edge value that has been flowed.
  /// @param ct The edge container this entry is held by.
  /// @param version_id The version identifier for @p val.  Zero if the full
  ///   producer. Otherwise a filtered version with the identifier previously
  ///   assigned in @ref add_local_notifier.
  /// @param b_caller_is_set_element Denotes whether the caller of this
  ///   method is set_element (i.e., the original data flow directly from
  ///   the producer task).
  /// @param b_caller_is_migration Denotes this is called when moving a
  ///   predecessor value for a successor migrated task.  Guards assertions
  ///   checking previous existence of value (to verify protocol correctness).
  ///
  /// @todo Use is_moveable<T> metafunction from runtime to guard generation of
  ///   move code paths for basic types, etc.
  //////////////////////////////////////////////////////////////////////
  template<typename Filter, typename ValueParam>
  void set_value(ValueParam&& val,
                 edge_container& ct,
                 unsigned int version_id,
                 bool b_caller_is_set_element = false,
                 bool b_caller_is_migration   = false)
  {
    const bool b_migration_enabled = ct.tg().migration_enabled();

    stapl_assert(!b_caller_is_set_element || version_id == 0,
      "b_caller_is_set_element and version_id != FULL");

    stapl_assert(m_remote_version.empty_notifications()
                || b_caller_is_set_element,
      "Found pending remote notifications and I'm not the producer location!");

    // Either filtered or full means signal requests can be notified.
    if (!this->signal_set())
    {
      // Using a pointer to handle case when edge_container backs a
      // pg input port initialized prior to executor construction.
      // notifications can't exist (as factory hasn't been called for setup
      // of internal consumers), but value may be set if flow already available.
      auto executor_ptr = ct.tg().executor_ptr();

      this->set_signal(executor_ptr, b_migration_enabled);
    }
    else
      stapl_assert(this->is_persistent() || this->empty_signal_notifications(),
        "m_b_signal_flow_set with pending SIGNAL notifications");

    // When producer calls set_value via tg::processed, if no local consumers
    // exists yet, there isn't a version entry yet.  Also may not exist when
    // migrating a consume value during task migration.
    // Otherwise, we expect there to be one and will assert otherwise.
    const bool b_expect_exist =
      !(b_caller_is_set_element || b_caller_is_migration);

    this->set_value_impl<Filter>(
      std::forward<ValueParam>(val), ct, version_id,
      b_caller_is_set_element, b_migration_enabled, b_expect_exist,
      typename std::is_same<
        Filter, detail::df_identity<stored_value_t>>::type());
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Used to trigger data flow in this entry for edges that are
  /// void.
  ///
  /// @param ct The @p edge_container holding this entry.
  ///
  /// Redirect to general set_value with default constructed value (this type
  /// may be unknown in the calling context, as this is a virtual function that
  /// can be called through @p edge_entry_base.
  //////////////////////////////////////////////////////////////////////
  void set_value(edge_container& ct) final
  {
    this->set_value<detail::df_identity<stored_value_t>>(
      stored_value_t(), ct, 0, true);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief The general evictability test for the edge_entry. Detects
  /// whether this entry is no longer needed on the location and
  /// can be removed.
  ///
  /// Verifies all consumers have been registered and notified (if this is the
  /// producer location) and that no local consumers are still using any version
  /// maintained by the entry.
  //////////////////////////////////////////////////////////////////////
  bool persistent_evictable(void) const
  {
    return this->m_unknown_consumer_cnt == 0
        && this->m_local_user_cnt == 0
        && empty_local_notifications()
        && m_remote_version.empty_notifications()
        && !m_remote_version.test_flushing();
  }


  ///////////////////////////////////////////////////////////////////////
  /// @brief Check whether this entry is no longer needed on the location and
  /// can be removed.
  ///
  /// Guarded to return false if the PARAGRAPH is persistent.  Eviction attempts
  /// in the edge container are disabled, allowing entry to remain until the
  /// destructor of the @p edge_container (as part of overall PARAGRAPH
  /// destruction).
  //////////////////////////////////////////////////////////////////////
  bool evictable(void) const
  {
    return this->persistent_evictable() && !this->is_persistent();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Extract all state necessary to migrate the producer state from this
  /// entry and move it to new execution location.
  ///
  /// @param other A reference to an empty notifier list.
  /// Move all remote notifiers into this. @p edge_container::migrate_entry will
  /// forward these notifiers there after invoking @p directory::migrate.
  ///
  /// @param ct_unknown_consumer_cnt A reference to the edge container's global
  /// unknown consumer count.  Decrement with entry's current unknown count,
  /// as these are now another location's responsibility to track.
  ///
  /// @return The original successor count specified for the producer task.
  ///
  /// @sa edge_container::migrate_entry
  //////////////////////////////////////////////////////////////////////
  size_t migrate(remote_notifier_list_t& other, int& ct_unknown_consumer_cnt)
  {
    stapl_assert(!this->is_persistent(),
      "Tried to migrate persistent edge_entry");

    // Before migrating the task or notifiers, reset the entry as if the
    // producer was never here (i.e., set m_unknown_consumer_cnt to 0, and
    // clear remote notifications). This is because we can poll in the
    // asyncs (and could potentially receive the task back or more notifiers)
    // Save the total_consumer_cnt before resetting the entry.
    //
    stapl_assert(this->m_unknown_consumer_cnt != defer_spec,
      "migration not supported for delayed successor specification yet");

    // ***NOTE***
    //
    // The size() call on remote_notifications list is non-constant time.
    // It's linear with the number of notifications.  This saves us a word
    // in all cache entries, with the only cost being this lookup when we
    // migrate.  Could flip on constant time for migration enabled
    // edge_containers / PARAGRAPHs if we thought this was a real problem.
    //
    const size_t total_consumer_cnt =
      this->m_unknown_consumer_cnt + m_remote_version.notifications().size();

    // This cast can go if we make entry.m_unknown_consumer_cnt of type int.
    stapl_assert(ct_unknown_consumer_cnt >= (int) this->m_unknown_consumer_cnt,
      "edge_container's aggregate unknown consumers less than entry's");

    ct_unknown_consumer_cnt -= this->m_unknown_consumer_cnt;

    this->m_unknown_consumer_cnt = 0;

    stapl_assert(other.empty(), "expected empty other");

    other.swap(m_remote_version.notifications());

    return total_consumer_cnt;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Reset data flow triggers prior to a persistent PARAGRAPH
  /// reinvocation.
  ///
  /// Reset the remote version list and all versions stored in the versions
  /// list.  Also invoke @p edge_entry_base::reset_values to reset the signal
  /// version list.
  //////////////////////////////////////////////////////////////////////
  void reset_values(void) final
  {
    edge_entry_base::reset_values();

    for (auto&& version : m_versions)
      version.clear_flow();

    m_remote_version.clear_flow();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if the full value of the producer task is currently
  /// available (i.e., data flow triggered) in this entry.
  //////////////////////////////////////////////////////////////////////
  bool full_value_set(void) const final
  {
    const versions_citer_t iter = this->iterator_lookup_version(FULL);

   if (iter == m_versions.end())
     return false;

   full_edge_version_entry_t const& full_version =
     down_cast<full_edge_version_entry_t const&>(*iter);

   stapl_assert(this->signal_set() || !full_version.test_flow(),
     "FULL set but not SIGNAL");

    return full_version.test_flow();
  }

private:
  /////////////////////////////////////////////////////////////////////
  /// @brief Scan version list and return a const_iterator to the version entry
  /// for given request_type value.
  ///
  /// @param req The request type (e.g., FILTERED, FULL) to search for in the
  /// versions list.
  ///
  /// This lookup may be preferable to the "get" version below, as it allows the
  /// caller to take different action based on whether or not entry exists.
  //////////////////////////////////////////////////////////////////////
  versions_citer_t iterator_lookup_version(edge_request_type req) const
  {
    return std::find_if(m_versions.begin(), m_versions.end(), compare_req(req));
  }


  /////////////////////////////////////////////////////////////////////
  /// @brief Scan version list and return a iterator to the version entry for
  /// given request_type value.
  ///
  /// @param req The request type (e.g., FILTERED, FULL) to search for in the
  /// versions list.
  ///
  /// This lookup may be preferable to the "get" version below, as it allows the
  /// caller to take different action based on whether or not entry exists.
  //////////////////////////////////////////////////////////////////////
  versions_iter_t iterator_lookup_version(edge_request_type req)
  {
    return std::find_if(m_versions.begin(), m_versions.end(), compare_req(req));
  }


  /////////////////////////////////////////////////////////////////////
  /// @brief Scan version list and return reference to the filtered
  /// version entry.  Create if non-existent.
  ///
  /// @param b_expected True if the caller expects the value be there
  /// (i.e., for correctness of protocol).  Guards conditional assert.
  //////////////////////////////////////////////////////////////////////
  filtered_edge_versions_t& get_filtered_versions(bool b_expected = false)
  {
    versions_iter_t iter = iterator_lookup_version(FILTERED);

    if (iter != m_versions.end())
      return down_cast<filtered_edge_versions_t&>(*iter);

    stapl_assert(!b_expected, "didn't find existing when b_expected");

    m_versions.push_front(*new filtered_edge_versions_t());

    return down_cast<filtered_edge_versions_t&>(*m_versions.begin());
  }


  /////////////////////////////////////////////////////////////////////
  /// @brief Scan version list and return const reference to the filtered
  /// version entry.
  ///
  /// All uses of this method expect the entry to be there. Assert if it isn't.
  //////////////////////////////////////////////////////////////////////
  filtered_edge_versions_t const& get_filtered_versions(void) const
  {
    stapl_assert(iterator_lookup_version(FILTERED) != m_versions.end(),
      "failed to find version entry");

    return down_cast<filtered_edge_versions_t const&>
      (*iterator_lookup_version(FILTERED));
  }


  /////////////////////////////////////////////////////////////////////
  /// @brief Scan version list and return reference to the full
  /// value entry. Create if non-existent.
  ///
  /// @param b_expected True if the caller expects the value be there
  /// (i.e., for correctness of protocol).  Guards conditional assert.
  //////////////////////////////////////////////////////////////////////
  full_edge_version_entry_t& get_full_version(bool b_expected = false)
  {
    versions_iter_t iter = iterator_lookup_version(FULL);

    if (iter != m_versions.end())
      return down_cast<full_edge_version_entry_t&>(*iter);

    stapl_assert(!b_expected, "didn't find existing when b_expected");

    m_versions.push_front(*new full_edge_version_entry_t());

    return down_cast<full_edge_version_entry_t&>(*m_versions.begin());
  }


  /////////////////////////////////////////////////////////////////////
  /// @brief Scan version list and return const reference to the full
  /// value entry.
  ///
  /// All uses of this method expect the entry to be there. Assert if it isn't.
  //////////////////////////////////////////////////////////////////////
  full_edge_version_entry_t const& get_full_version(void) const
  {
    stapl_assert(iterator_lookup_version(FULL) != m_versions.end(),
      "failed to find version entry");

    return down_cast<full_edge_version_entry_t const&>
      (*iterator_lookup_version(FULL));
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Called when eviction is checked polymorphically through
  /// @p edge_entry_base::evictable. Redirect to @p edge_entry::evictable.
  ///
  /// @sa evictable.
  //////////////////////////////////////////////////////////////////////
  bool poly_evictable(void) const final
  {
    return this->evictable();
  }
}; // class edge_entry


//////////////////////////////////////////////////////////////////////
/// @brief Provides trivial iterator interface to an @ref edge_entry
/// pointer and avoids serialization of object.  Used in data flow
/// fire (@ref edge_container::receive_value) to elide relookup in
/// value cache.
///
/// @todo Remove usage if/when ARMI allows bitwise() style annotation
/// on arguments passed to an RMI (to avoid serialization of pointed
/// to object when all that is desired is the address).
//////////////////////////////////////////////////////////////////////
template<typename T>
class edge_entry_wrapper
{
private:
  edge_entry<T>* m_edge_entry_ptr;

public:
  edge_entry_wrapper(edge_entry<T>* edge_entry_ptr)
    : m_edge_entry_ptr(edge_entry_ptr)
  { }

  edge_entry<T>* operator->()
  { return m_edge_entry_ptr; }

  edge_entry<T>& operator*()
  { return *m_edge_entry_ptr; }

  void define_type(typer& t)
  { t.member(bitwise(t)); }

  edge_entry<T>* pointer(void)
  { return m_edge_entry_ptr; }

}; // class edge_entry_wrapper

} // namespace detail

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_EDGE_ENTRY_HPP


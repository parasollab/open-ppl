/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_EDGE_ENTRY_BASE_HPP
#define STAPL_PARAGRAPH_EDGE_ENTRY_BASE_HPP

#include <bitset>

#include <stapl/utility/hash_fwd.hpp>
#include <boost/intrusive/list.hpp>
#include <boost/intrusive/unordered_set.hpp>

#include <stapl/paragraph/edge_container/utility.hpp>
#include <stapl/paragraph/edge_container/edge_version_entry.hpp>
#include <stapl/utility/empty_class.hpp>

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief The stored element type of @p edge_container.  One instance of
/// this class or a derived class exists for each producer task, both on the
/// location where it executes as well as on all locations with active
/// consumers.
///
/// Can be directly instantiated for consumer locations when there are only
/// signal consumers.  Alternatively, can be instantiated as base class of
/// instantiation of @p edge_entry class template object.
///
/// Lack of template parameter on edge type on this entry type allows the
/// @p edge_container to support multiple edge value types within the same
/// PARAGRAPH, an important feature for many dependence graph patterns.
///
/// @ingroup pgEdgeEntry
///
/// @todo Type guard delayed successor count functionality (only used for some
/// some dynamic cases such as the while loop pattern.  Even without type guard
/// can optimized: coalesce m_unknown_consumer_cnt and m_seen_consumers with
/// high order bit denoting how to interpret.
///
/// @todo This class really serves two purposes: type erased, polymorphic access
/// and direct instantiation for signals.  It is probably cleaner to separate
/// these duties, introduces a signal_entry inheriting from this class. This
/// would make virtual methods such as @p set_value pure as they should be.
//////////////////////////////////////////////////////////////////////
class edge_entry_base
  : public boost::intrusive::unordered_set_base_hook<
      boost::intrusive::link_mode<boost::intrusive::safe_link>>
{
private:
  typedef boost::intrusive::unordered_set_base_hook<
    boost::intrusive::link_mode<
      boost::intrusive::safe_link>>           hook_type;

protected:
  /// @brief Task identifier of the produce task this entry refers to.
  const std::size_t                             m_tid;

  /// @brief The current level of consumption on location where this entry
  /// exists.
  edge_request_type                             m_request_state;

  /// @brief Count of consumers which are still unaccounted for. Only
  /// initialized on producer location, where the edge_entry will be kept in
  /// edge_container at least until all consumers have registered, decrementing
  /// this field to 0. Value of @p defer_spec, denotes location is set as
  /// producer location, but successor count will be set separately
  /// @sa delayed_set_num_consumers
  size_t                                        m_unknown_consumer_cnt;

  /// @c true if there is only one consumer.
  bool                                          m_single_consumer;

  /// @brief Tracks the number of consumers that have registered at producer
  /// location when m_unknown_consumer_cnt is set to defer_spec.
  size_t                                        m_seen_consumers;

  /// @brief track of consumer reference to version contained in this entry.
  /// Used to help manage the lifetime of the object.
  size_t                                        m_local_user_cnt;

  /// @brief Tracks whether corresponding PARAGRAPH is persistent.
  const bool                                    m_b_persistent;

  /// @brief Is the actual object instantiated @p edge_entry_base or
  /// @p edge_entry<T>.  Used to detect if we need to upgrade the entry based
  /// to @p edge_entry object.
  ///
  /// @sa edge_container::lookup_or_insert
  const bool                                    m_b_signal_only;

#ifndef STAPL_NDEBUG
  /// @brief Tracks whether consumer tasks at this location registered before
  /// the producer was registered.  If so, their remote notifiers may arrive out
  /// of order.  In debug mode, we need to track this for assertion.
  ///
  /// @sa remote_edge_version_entry::add_notifier
  bool                                          m_b_consumers_before_producer;
#endif

private:
  /// @brief Notifier used for signal consumers.
  typedef boost::function<
    void (executor_base&)>                      signal_notifier_t;

  /// @brief Notifier version list for signal consumers.
  typedef edge_version_entry<
    void, signal_notifier_t, empty_class>       signal_edge_version_entry_t;

protected:
  /// @brief The notifier list for all local signal consumers on this location.
  signal_edge_version_entry_t                   m_signal_version;

private:
  edge_entry_base& operator=(edge_entry_base const&) = delete;

  //////////////////////////////////////////////////////////////////////
  /// @brief This method exists separately from @p evictable to give us some
  /// flexibility in how we call it.  We need runtime dispatch in some contexts
  /// but know the full type of the edge entry in other contexts and prefer to
  /// avoid the virtual table lookup.
  ///
  /// @todo Check whether this entry is no longer needed on the location and
  /// can be removed.
  ///
  /// @sa evictable
  //////////////////////////////////////////////////////////////////////
  virtual bool poly_evictable(void) const
  {
    return m_unknown_consumer_cnt == 0
        && m_local_user_cnt == 0
        && this->empty_signal_notifications()
        && !this->is_persistent();
  }

public:
  explicit
  edge_entry_base(std::size_t tid, bool b_persistent, bool b_signal_only = true)
    : m_tid(tid),
      m_request_state(NONE),
      m_unknown_consumer_cnt(0),
      m_single_consumer(false),
      m_seen_consumers(0),
      m_local_user_cnt(0),
      m_b_persistent(b_persistent),
      m_b_signal_only(b_signal_only)
#ifndef STAPL_NDEBUG
      , m_b_consumers_before_producer(false)
#endif
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called in @ref Task::operator() (via @ref lazy_edge_reference)
  /// by a consumer identity operation to avoid needless copy if it can
  /// have a version stored in @ref edge_version_storage.
  //////////////////////////////////////////////////////////////////////
  bool stealable(void) const
  {
    return m_unknown_consumer_cnt == 0 && m_local_user_cnt == 1;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called when attempting to use object managed by edge entry
  /// as outgoing edge of paragraph.  @p m_local_user_cnt should be zero
  /// as result notifications do not increment this variable.
  //////////////////////////////////////////////////////////////////////
  bool out_edge_stealable(void)
  {
    return m_unknown_consumer_cnt == 0 && m_local_user_cnt == 0;
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief A move constructor invoked when upgrading a signal only entry
  /// to a full value @ref edge_entry.
  ///
  /// Call move constructor of signal version entry as well placing this new
  /// entry in the @ref edge_container in place of @p other.
  ///
  /// This is used when we upgrade an object of type edge_entry_base to that
  /// of edge_entry in the edge_container upon the reception of value consumers
  /// or the initialization of the producer task on this location.
  //////////////////////////////////////////////////////////////////////
  edge_entry_base(edge_entry_base&& other)
    : m_tid(other.m_tid),
      m_request_state(other.m_request_state),
      m_unknown_consumer_cnt(other.m_unknown_consumer_cnt),
      m_seen_consumers(other.m_seen_consumers),
      m_local_user_cnt(other.m_local_user_cnt),
      m_b_persistent(other.m_b_persistent),
      m_b_signal_only(false),
#ifndef STAPL_NDEBUG
      m_b_consumers_before_producer(false),
#endif
      m_signal_version(std::move(other.m_signal_version))
  {
    stapl_assert(m_unknown_consumer_cnt == 0
                 && m_seen_consumers == 0
                 && m_local_user_cnt == 0,
      "edge_entry_base move constr found unexpected state");

    // This entry inherits the other's place in edge_container's value cache.
    hook_type::swap_nodes(other);

    stapl_assert(!other.is_linked(), "move constructor, other is linked");
    stapl_assert(this->is_linked(),  "move constructor, this isn't linked");
  }

public:
  STAPL_USE_MANAGED_ALLOC(edge_entry_base)


  virtual
  ~edge_entry_base()
  {
    stapl_assert(
      m_unknown_consumer_cnt == 0
        && m_local_user_cnt == 0
        && this->empty_signal_notifications(),
      "~edge_entry_base: wasn't evictable..."
    );
  }


  virtual bool is_basic_edge_entry(void) const
  { return false; }


  bool is_signal_only(void) const
  {
    return m_b_signal_only;
  }


  bool is_persistent(void) const
  {
    return m_b_persistent;
  }


  std::size_t tid(void) const
  {
    return m_tid;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Freestanding function that defines @p operator== in terms of
  /// the entries' respective task identifiers.
  //////////////////////////////////////////////////////////////////////
  friend bool operator==(edge_entry_base const& lhs, edge_entry_base const& rhs)
  {
     return lhs.m_tid == rhs.m_tid;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Interface required for Boost.Intrusive unordered set (the
  /// container edge entries are stored in), to map an entry to it's hashed
  /// key for insertion.
  ///
  /// @param entry An edge entry to compute it's hash for set insertion.
  /// @return The hash_value corresponding to @p entry.
  //////////////////////////////////////////////////////////////////////
  friend std::size_t hash_value(edge_entry_base const &entry)
  {
    return boost::hash_value(entry.m_tid);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Check whether this entry is no longer needed on the location and
  /// can be removed.
  ///
  /// For @ref edge_entry_base, this is a simple redirect to the polymorphic
  /// eviction implementation, as we may need to dispatch to an instance
  /// @p edge_entry template, without knowing it's edge data type @p T.
  //////////////////////////////////////////////////////////////////////
  bool evictable(void) const
  {
    return this->poly_evictable();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Inform this edge entry that the producer task has been initialized
  /// on this location.
  ///
  /// @param consumer_cnt The number of consumers as specified by the
  /// corresponding @p add_task call. Possible value is @p defer_spec, denoting
  /// that this count will be specified later.
  ///
  /// @sa delayed_set_num_consumers
  //////////////////////////////////////////////////////////////////////
  void set_as_producer(size_t consumer_cnt)
  {
    stapl_assert(!this->local_producer_initialized(),
      "edge_entry_base::set_as_producer() local producer already initialized");

    // In current impl,non local consumers notifications can't have arrived
    // yet since I haven't given the directory my location.  In future:
    //   entry.m_unknown_consumer_cnt -= entry.consumer_locations.size();
    m_unknown_consumer_cnt = consumer_cnt;

    m_single_consumer = (consumer_cnt==1);

#ifndef STAPL_NDEBUG
    // Set flag if there were any consumers initialized at this location prior
    // to this initialization of the producer task here.  Marks that ordering
    // of remote notifiers may be broken.  That's ok, just be less restrictive
    // on test of upgrade bit in edge_entry::add_remote_notifier().
    m_b_consumers_before_producer =
      !this->empty_signal_notifications() || m_local_user_cnt;
#endif
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Set the number of successors after @p delay_spec was passed
  /// to @p set_as_producer.
  ///
  /// @param consumer_cnt The out-degree of the producer task in the PARAGRAPH.
  //////////////////////////////////////////////////////////////////////
  void delayed_set_num_consumers(size_t consumer_cnt)
  {
    stapl_assert(consumer_cnt != defer_spec,
      "Passed defer_spec to delayed succ_cnt specification");

    stapl_assert(m_unknown_consumer_cnt == defer_spec,
      "set_unknown_consumers called when consumers already set");

    stapl_assert(consumer_cnt >= m_seen_consumers,
      "more consumers already seen than expected");

    m_unknown_consumer_cnt = consumer_cnt - m_seen_consumers;

    m_single_consumer = (consumer_cnt==1);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Reduce the unregistered consumer count.
  ///
  /// Called when previous consumption on a consumer location covers a request,
  /// meaning no additional notifications are required.  A simple decrement of
  /// out-degree counter is sufficient.
  //////////////////////////////////////////////////////////////////////
  void decrement_unknown_consumers(void)
  {
    stapl_assert(this->local_producer_initialized(),
      "edge_entry_base::decrement_unknown_consumers() on non producer loc");

    if (m_unknown_consumer_cnt == defer_spec)
    {
      ++m_seen_consumers;
    }
    else
    {
      --m_unknown_consumer_cnt;
    }
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Increase reference count of local proxies in use by consumer tasks
  /// and backed by a version in this edge entry.
  ///
  /// @sa edge_accessor
  //////////////////////////////////////////////////////////////////////
  void increment_local_consumers(void)
  {
    ++m_local_user_cnt;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Decrease reference count of local proxies in use by consumer tasks
  /// and backed by a version in this edge entry.
  ///
  /// @sa edge_accessor
  //////////////////////////////////////////////////////////////////////
  void decrement_local_consumers(void)
  {
    stapl_assert(m_local_user_cnt != 0,
      "decrement_local_consumers found m_local_user_cnt == 0");

    --m_local_user_cnt;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Check if the producer task for edge_entry is registered on this
  /// location.
  ///
  /// Sufficient to check if m_unknown_consumer_cnt has been initialized.
  //////////////////////////////////////////////////////////////////////
  bool local_producer_initialized() const
  {
    return m_unknown_consumer_cnt;
  }

  bool has_single_consumer(void) const noexcept
  {
    return m_single_consumer;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Redirect check signal flow request to the signal version
  /// notifier list.
  //////////////////////////////////////////////////////////////////////
  bool signal_set() const
  {
    return m_signal_version.test_flow();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Check if full value of producer has been set on this location.
  ///
  /// For edge_entry_base objects, is always false, as the producer isn't
  /// initialized and only signal consumers can have registered.
  //////////////////////////////////////////////////////////////////////
  virtual bool full_value_set(void) const
  {
    return false;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Reset data flow triggers prior to a persistent PARAGRAPH
  /// reinvocation.
  ///
  /// For edge_entry_base objects, the only version that can exist to reset
  /// is the signal notifier list.
  //////////////////////////////////////////////////////////////////////
  virtual void reset_values(void)
  {
    m_signal_version.clear_flow();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Set data flow for signal consumers by notifying the signal version
  /// notifier list
  ///
  /// @param executor_ptr The executor associated with the given PARAGRAPH.
  /// @param b_migration_enabled Denotes whether the PARAGRAPH has task
  /// migration support enabled.
  //////////////////////////////////////////////////////////////////////
  void set_signal(executor_base* executor_ptr, const bool b_migration_enabled)
  {
    m_signal_version.set_value(
      executor_ptr, b_migration_enabled, is_persistent());
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief The set value signature used by the producer task via
  /// @p task_graph::processed when the return type is void.
  ///
  /// This call should only occur on the producer location,
  /// where edge_entry overrides this definition.
  ///
  /// @sa edge_entry::set_value
  //////////////////////////////////////////////////////////////////////
  virtual void set_value(edge_container&)
  {
    stapl_assert(0, "set_value() called on edge_entry_base");
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Add notifier to remote location for signaling.
  ///
  /// This call should only occur on the producer location, where edge_entry
  /// overrides this definition.
  ///
  /// @sa edge_entry::add_remote_signal_notifier
  //////////////////////////////////////////////////////////////////////
  virtual void add_remote_signal_notifier(edge_container&/*ct*/, size_t/*loc*/)
  {
    stapl_assert(0, "entry_base::add_remote_signal_notifier() invoked");
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Cleanup all notification associated with this edge_entry.
  ///
  /// For @ref edge_entry_base instantiations, the only notifications for
  /// signals.
  //////////////////////////////////////////////////////////////////////
  virtual void cleanup_notifications(void)
  {
    m_signal_version.cleanup_notifications();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Called when adding a local notification to determine whether a
  /// a consumption request is covered by previous requests or whether a new
  /// remote notifier needs to be generated and sent to the producer location.
  ///
  /// @param req             The type of request that has just been added.
  /// @param b_flow          Whether data flow has already been triggered on
  ///                        this consumer location for @p req.
  /// @param b_partial_added If @p req is partial, denotes if this request
  ///                        added a new version (i.e., filter) on this location
  ///                        for the producer task value.
  ///
  /// @sa add_signal_notifier
  /// @sa edge_entry::add_local_notifier
  //////////////////////////////////////////////////////////////////////
  df_add_status compute_request_status(const edge_request_type req,
                                       const bool b_flow,
                                       const bool b_partial_added = false)
  {
    // If I notified or could have but queued to remain notification ordering,
    // the request is definitely covered.  No need to fire off remote req.
    //
    // In theory could try eviction... (if all we have is SIGNALS)...
    if (b_flow)
      return REMOTE_REQ_COVERED;

    // uninitialized and/or remote producer, and no previous consumers.
    if (this->m_request_state == NONE)
    {
      this->m_request_state = req;

      return SEND_NOTIFIER_REQ;
    }

    // NOTE - In the future, if we synthesize multiple filter functors, this
    // becomes more complicated and interesting than a simple upgrade.  In
    // view-speak, if we constrain ourselves to functors that simply restrict
    // the domain of collections, we need to union the domains and later
    // transform the functors when servicing value() requests.
    //
    // Previous consumption request exists and covers this one.
    //
    if (req < this->m_request_state)
      return REMOTE_REQ_COVERED;

    if (req == this->m_request_state)
    {
      if (req != FILTERED)
        return REMOTE_REQ_COVERED;

      return b_partial_added ? SEND_NOTIFIER_REQ : REMOTE_REQ_COVERED;
    }

    // Previous request needs to be upgraded to cover this.
    //
    this->m_request_state = req;

    return SEND_NOTIFIER_REQ;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Add a signal notifier for a local task.  Return the request
  /// level status information by calling @ref compute_request_status.
  ///
  /// @param executor The executor associated with the PARAGRAPH.  Passed to
  /// notifier if it is immediately called (i.e., the signal flow has already
  /// been set.
  ///
  /// @param notifier The signal notifier for the successor task.
  ///
  /// @sa edge_container::setup_signal_flow
  //////////////////////////////////////////////////////////////////////
  df_add_status
  add_signal_notifier(executor_base& executor, signal_notifier_t notifier)
  {
    m_signal_version.add_notifier(notifier, executor, this->is_persistent());

    return this->compute_request_status(SIGNAL, m_signal_version.test_flow());
  }


protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if the the @p edge_entry contains any signal
  /// notifications.
  //////////////////////////////////////////////////////////////////////
  bool empty_signal_notifications(void) const
  {
    return m_signal_version.empty_notifications();
  }
}; // class edge_entry_base


//////////////////////////////////////////////////////////////////////
/// @brief Minimal entry class used when edge initialized without an
/// @ref edge_container.
//////////////////////////////////////////////////////////////////////
template<typename T>
class basic_edge_entry
  : public edge_entry_base,
    public edge_version_storage<typename df_stored_type<T>::type>
{
public:
  basic_edge_entry(std::size_t tid)
    : edge_entry_base(tid, false, false)
  { this->increment_local_consumers(); }

  bool is_basic_edge_entry(void) const override
  { return true; }

  using edge_version_storage<typename df_stored_type<T>::type>::set_value;

  STAPL_USE_MANAGED_ALLOC(basic_edge_entry)
};


//////////////////////////////////////////////////////////////////////
/// @brief Comparator required by methods of Boost.Intrusive unordered_set
/// to search for elements. Allows lookups based on the edge entry's task
/// identifier field.
///
/// @ingroup pgEdgeEntry
///
/// @sa edge_container::lookup
/// @sa edge_container::lookup_or_insert
//////////////////////////////////////////////////////////////////////
struct entry_compare
{
  bool operator()(edge_entry_base const& lhs, std::size_t rhs) const
  {
    return lhs.tid() == rhs;
  }

  bool operator()(std::size_t lhs, edge_entry_base const& rhs) const
  {
    return lhs == rhs.tid();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object implementing the disposer concept defined by
/// Boost.Intrusive.
///
/// @ingroup pgEdgeEntry
///
/// Intrusive containers do not manage the lifetime of the objects they
/// contain.  This is left to the container's user to manage.  The disposer
/// callback functor allows the user to do this during container operations
/// where object deletion may be desired (i.e., clear()).  In the edge_entry
/// context, this disposer is a simple delete call.
///
/// Function operator signatures restricted to @ref edge_entry
/// to guard against inadvertent use in other contexts.
///
/// Only called for edge_entry when in persistent pg mode.
/// Otherwise, this will happen in try_eviction.
//////////////////////////////////////////////////////////////////////
struct entry_disposer
{
  void operator()(edge_entry_base* entry_ptr) const
  {
    delete entry_ptr;
  }
};

} // namespace detail

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_EDGE_ENTRY_BASE_HPP


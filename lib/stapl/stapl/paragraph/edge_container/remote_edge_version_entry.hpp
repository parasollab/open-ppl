/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_REMOTE_EDGE_VERSION_ENTRY_HPP
#define STAPL_PARAGRAPH_REMOTE_EDGE_VERSION_ENTRY_HPP

namespace stapl {

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Storage location for @ref remote_notifier class, mainly
/// so that @ref remote_edge_version_entry can access this information
/// for assertion in a type erased (i.e., lighter compile time) manner.
//////////////////////////////////////////////////////////////////////
class location_storage
{
private:
  /// @brief The location this remote notifier refers to.
  location_type m_location;

public:
  location_storage(location_type loc)
    : m_location(loc)
  { }

  size_t location(void) const
  {
    return m_location;
  }
}; // class location_storage


//////////////////////////////////////////////////////////////////////
/// @brief Notifier invoker passed by @ref remote_edge_version_entry
/// to @ref notifier_list base class.
///
/// Explicit functor instead of lambda to reduce compilation pressure
/// and code dupe (multiple usage sites).
//////////////////////////////////////////////////////////////////////
template<typename FullValueAccess, typename Notifier>
class remote_notifier_invoker
{
private:
  edge_container&        m_ct;
  std::size_t            m_tid;
  FullValueAccess const& m_full_value;
  bool                   m_b_moveable;

public:
  remote_notifier_invoker(edge_container& ct,
                          std::size_t tid,
                          FullValueAccess const& full_value,
                          bool b_moveable)
    : m_ct(ct), m_tid(tid), m_full_value(full_value), m_b_moveable(b_moveable)
  { }

  void operator()(Notifier const& notifier) const
  {
    notifier(m_ct, m_tid, m_full_value(), false, m_b_moveable);
  }
}; // class remote_notifier_invoker


//////////////////////////////////////////////////////////////////////
/// @brief Compares location member to @ref remote_notifier base class
/// @ref location_storage (assertion purposes).
///
/// Explicit functor instead of lambda to reduce compilation pressure.
//////////////////////////////////////////////////////////////////////
class notifier_loc_compare
{
private:
  location_type m_location;

public:
  notifier_loc_compare(location_type loc)
    : m_location(loc)
  { }

  bool operator()(location_storage const& entry) const
  {
    return entry.location() == m_location;
  }
}; // class notifier_loc_compare


//////////////////////////////////////////////////////////////////////
/// @brief Maintains list of edge notifications for locations with remote
///   consumers.
/// @ingroup pgEdgeEntry
///
/// @tparam T        The value type produced by the producer task.
/// @tparam Notifier Edge notifiers for remote consumers locations registered
///                  with the producer.  Currently, an instantiation of class
///                  template @p edge_notifier_wrapper.
/// @tparam Hook     The Boost.Intrusive container entry hook type used to
///                  place this version to store this version and notifiers
///                  in a list with other versions held by the @p edge_entry.
///
/// @sa edge_notifier_wrapper
///
/// Currently, objects from this class template are only stored on the producer
/// location.  Unlike local notifications (i.e., signal, filtered, or full),
/// no value data member is held.  The produced value is always available on
/// producer location after producer task execution until all successors have
/// been notified and local successors have finished execution. Hence, the
/// @p edge_entry just passes a reference to this value when calling
/// @p add_notifier and @p set_value.
///
/// @todo Expose enough typing information that we can upgrade existing
/// entries, effectively compacting the list and reducing communication.
/// The current implementation is correct and represents the case when the
/// later requests arrive after the existing has been flushed.  As long as
/// we maintain order, the consumer location is none the wiser.  Might be
/// able to get away with a bit in entry signaling current request level.
/// Also consider if we want an ordered container to minimize scan times.
/// Bottom line, compaction via req upgrade costs us local cycles
/// (either O(out-degree) or O(lg(out-degree) plus some memory) to avoid
/// communication.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Notifier, typename Hook>
class remote_edge_version_entry
  : public notifier_list<Notifier>,
    public Hook
{
private:
  using base_t = notifier_list<Notifier>;

  /// @brief signature of function operator for remote notifiers.
  using mem_fun_t =
    void (Notifier::*)(edge_container&, std::size_t, T const&, bool) const;

public:
  using entry_t         = typename base_t::notifier_entry_t;
  using notifier_list_t = typename base_t::notifier_list_t;

  STAPL_USE_MANAGED_ALLOC(remote_edge_version_entry)


  //////////////////////////////////////////////////////////////////////
  /// @brief Add a new remote notifier, invoking it if flow has previously been
  /// set.
  //
  /// @param notifier     New remote notifier to add to list.
  /// @param tid          The task identifier of the associated @ref edge_entry.
  /// @param ct           The edge_container associated with notifier.
  /// @param full_value   The task identifier of the associated @ref edge_entry.
  /// @param b_full_flow  The current flow state of the value produced by
  ///                     @p tid (i.e., is @p full_value valid?).
  /// @param b_persistent True if corresponding PARAGRAPH is persistent.
  ///                     False otherwise.
  /// @param b_upgrade    Is this new notifier an upgrade of a previous request
  ///                     (i.e., a FULL after a previous SIGNAL request).
  /// @param b_moveable   Whether function allowed to move out the value to
  ///                     service a single remote consumer.
  ///
  /// @param b_consumers_before_producer True if consumers were registered on
  /// this location (creating an @ref edge_entry) prior to the initialization of
  /// the producer.  This parameter is only passed in debug mode to allow an
  /// assert to be properly guarded by a corner case caused by unconditional
  /// insertion of a remote notifier on the initial execution location (this
  /// simplifies the migration protocol).  Discussed in more detail in
  /// @ref edge_container::setup_flow.
  ///
  /// Invokes base member @ref notifier_list::add_notifier_impl after
  /// constructing a notifier invoker lambda using the input parameters.
  //////////////////////////////////////////////////////////////////////
  template<typename FullValueAccess>
  void add_notifier(Notifier const& notifier,
                    std::size_t tid,
                    edge_container& ct,
                    FullValueAccess const& full_value,
                    const bool b_full_flow,
                    const bool b_persistent,
                    const bool b_upgrade,
                    const bool b_moveable
#ifndef STAPL_NDEBUG
                    , const bool b_consumers_before_producer
#endif
                    )
  {
    // Verify that we aren't getting a repeated, non upgrade request from the
    // same consumer location.  The consumer side protocol should intercept and
    // avoid these.
    stapl_assert(b_upgrade
      || this->m_notifications.end() == std::find_if(
         this->m_notifications.begin(), this->m_notifications.end(),
         notifier_loc_compare(notifier.location()))
      || (notifier.location()
          == ct.get_location_id() && b_consumers_before_producer),
      "loc already registeried a notifier & !b_upgrade");

    if (b_full_flow)
      this->set_flow();

    this->add_notifier_impl(
      remote_notifier_invoker<FullValueAccess, Notifier>(
        ct, tid, full_value, b_moveable),
      notifier, b_persistent);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief This method is called by the corresponding @ref edge_entry to
  /// notify this edge version of the produced value so that it can flush
  /// its notification list.
  ///
  /// @param ct           The edge container holding the entry this list of
  ///                     remote notifications refers to.
  /// @param tid          The task identifier of the associated @p edge_entry.
  /// @param full_value   The value produced by task @p tid.
  /// @param b_persistent True if corresponding PARAGARAPH is persistent.
  ///                     False otherwise.
  /// @param b_moveable   Whether function allowed to move out the value to
  ///                     service a single remote consumer.
  ///
  /// Invokes base member @ref notifier_list::add_notifier_impl after
  /// constructing a notifier invoker lambda using the input parameters.
  /// @sa notifier_list::flush_notifications
  //////////////////////////////////////////////////////////////////////
  template<typename FullValueAccess>
  void set_value(edge_container& ct, std::size_t tid,
                 FullValueAccess const& full_value,
                 bool b_persistent, bool b_moveable)
  {
    this->flush_notifications(
      remote_notifier_invoker<FullValueAccess, Notifier>(
        ct, tid, full_value, b_moveable),
      b_persistent);

    this->set_flow();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Enable access to notification list.
  ///
  /// This method exists solely to support migration.  The associated
  ///   @ref edge_entry performs a swap on with this list and returns the
  ///   elements to @ref edge_container::migrate_entry so that these notifiers
  ///   can be forwarded to the task's new execution location.
  //////////////////////////////////////////////////////////////////////
  notifier_list_t& notifications(void)
  { return this->m_notifications; }
}; // class remote_edge_version_entry

} // namespace detail

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_REMOTE_EDGE_VERSION_ENTRY_HPP

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_NOTIFIER_LIST_HPP
#define STAPL_PARAGRAPH_NOTIFIER_LIST_HPP

namespace stapl {

namespace detail {

template<typename, typename>
class notifier_entry;

template<typename, typename, typename>
class remote_edge_version_entry;


//////////////////////////////////////////////////////////////////////
/// @brief Function object implementing the disposer concept defined by
///   Boost.Intrusive.
/// @ingroup pgEdgeNotifiers
///
/// Intrusive containers do not manage the lifetime of the objects they
/// contain.  This is left to the container's user to manage.  The disposer
/// callback functor allows the user to do this during container operations
/// where object deletion may be desired (i.e., clear()).  In the version
/// context, this disposer is a simple delete call.
///
/// Function operator signatures restricted to @p notifier_entry and
/// @ref remote_edge_version_entry to guard against inadvertent use in
/// other contexts.
//////////////////////////////////////////////////////////////////////
struct notifier_entry_disposer
{
  template<typename Notifier, typename Hook>
  void operator()(notifier_entry<Notifier,Hook>* entry_ptr) const
  {
    delete entry_ptr;
  }

  template<typename T, typename Notifier, typename Hook>
  void operator()(remote_edge_version_entry<T, Notifier, Hook>* entry_ptr) const
  {
    delete entry_ptr;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief An entry in the notification list held by @p notifier_list. Inherit
/// the @p Notifier function operator, add @p Hook for storage in a
/// Boost.Intrusive container, and define pool allocation usage via
/// STAPL_USE_MANAGED_ALLOC.
/// @ingroup pgEdgeNotifiers
//////////////////////////////////////////////////////////////////////
template<typename Notifier, typename Hook>
class notifier_entry
  : public Notifier,
    public Hook
{
private:
  notifier_entry(notifier_entry const&) = delete;

  notifier_entry& operator=(notifier_entry const&) = delete;

public:
  notifier_entry(Notifier notifier)
    : Notifier(std::move(notifier))
  { }

  STAPL_USE_MANAGED_ALLOC(notifier_entry)
};


//////////////////////////////////////////////////////////////////////
/// @brief Base class of @ref notifier_list which holds data members and
/// operation not dependent on template parameters (avoid code bloat).
/// @ingroup pgEdgeNotifiers
//////////////////////////////////////////////////////////////////////
class notifier_list_base
{
private:
  /// @brief Tracks whether data flow of value associated with notifier has
  /// been triggered.
  bool m_b_flow_set;

  /// @brief Set when a flush of the notifier list is in process.  Enables us
  /// a maintain the FIFO ordering of notifier processing.  Not necessary for
  /// correctness, but makes thing easier to debug and to detect evictability
  /// of the corresponding @p edge_entry
  bool m_b_flushing;

public:
 notifier_list_base()
    : m_b_flow_set(false),
      m_b_flushing(false)
  { }

  void clear_flow(void)
  {
    m_b_flow_set = false;
  }

protected:
  void set_flow(void)
  {
    m_b_flow_set = true;
  }

public:
  bool test_flow(void) const
  {
    return m_b_flow_set;
  }

protected:
  void clear_flushing(void)
  {
    m_b_flushing = false;
  }

  void set_flushing(void)
  {
    m_b_flushing = true;
  }

public:
  bool test_flushing(void) const
  {
    return m_b_flushing;
  }
}; // class notifier_list_base


//////////////////////////////////////////////////////////////////////
/// @brief Provides storage for notifiers associated with a edge version
/// of a task.  Defines operations to add a new notifier and flush
/// (i.e., invoke) notifiers when the corresponding data flow is triggered.
/// @ingroup pgEdgeNotifiers
///
/// @tparam Notifier The notifier type stored by the list.  No specific function
/// operator signature is assumed; this detail is abstracted by the
/// @p NotifierInvoker template parameter of methods that may need invoke
/// notifier objects.
///
/// @sa edge_version_entry
/// @sa remote_edge_version_entry
///
/// @todo In flush notifications, guard creation of temporary notifier list
/// and associated operations by a static persistency type to avoid overhead.
///
/// @todo It may be more advantageous to add to the front of the notifier list
/// instead of using push_back to avoid unneeded list traversal.  However, this
/// has ramifications for notification order which may have undesired effects
/// on the order in which tasks are presented to the executor / scheduler.
//////////////////////////////////////////////////////////////////////
template<typename Notifier>
struct notifier_list
  : public notifier_list_base
{
protected:
  typedef notifier_entry<
    Notifier,
    boost::intrusive::slist_base_hook<>
  >                                             notifier_entry_t;

  typedef boost::intrusive::slist<
    notifier_entry_t,
    boost::intrusive::constant_time_size<false>,
    boost::intrusive::cache_last<true>
  >                                             notifier_list_t;

  notifier_list& operator=(notifier_list const&);

  notifier_list(notifier_list const&);

protected: // Allow access remote_notifications, migration.
  notifier_list_t                               m_notifications;

public:
  notifier_list(void) = default;

  notifier_list(notifier_list&& other)
    : notifier_list_base(other)
  {
    // Take any pending notifications from other into this entry.
    m_notifications.swap(other.m_notifications);

    stapl_assert(other.m_notifications.empty(), "move constr, not empty?");
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @brief Invoke all notifier in the list, deleting them afterwords as part
  /// of incremental PARAGRAPH destruction, unless persistency is enabled.
  ///
  /// @param invoker      Higher order function object which is responsible for
  ///                     calling the notifier with whatever arguments are
  ///                     necessary.
  /// @param b_persistent Denotes whether persistency is enabled in the
  ///                     associated PARAGRAPH. Guards the deletion of a
  ///                     notifier after it is invoked.
  ///
  /// This protected method is used by derived classes to implement their
  /// @p set_value methods (which are called by the edge_entry when the
  /// corresponding data flow is locally triggered.
  //////////////////////////////////////////////////////////////////////
  template<typename NotifierInvoker>
  void flush_notifications(NotifierInvoker const& invoker,
                           const bool b_persistent)
  {
    stapl_assert(!this->test_flushing(), "flush already in progress");

    if (m_notifications.empty())
      return;

    // Without this to disable evictability, nested requests to this entry
    // (possible from async below) may evict it, since conditions will be met.
    // This function's try_eviction will be attempted on an already evicted tid
    // If we guard attempt with a find check, this ok; however it is much
    // easier to test assertions knowing it should be there.
    this->set_flushing();

    notifier_list_t tmp_notifier_list;

    while (!m_notifications.empty())
    {
      notifier_entry_t& notifier_entry = m_notifications.front();

      m_notifications.pop_front();

      invoker(notifier_entry);

      if (b_persistent)
        tmp_notifier_list.push_back(notifier_entry);
      else
        delete &notifier_entry;
    }

    if (b_persistent)
      m_notifications.swap(tmp_notifier_list);

    this->clear_flushing();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Add a new notifier to the list, possibly invoking it immediately
  /// if the associated data flow was previously triggered.
  ///
  /// @param invoker      Higher order function object which is responsible for
  ///                     calling the notifier with whatever arguments are
  ///                     necessary.
  /// @param notifier     The edge notifier that tied to the data flow this
  ///                     notifier list is a part of.
  /// @param b_persistent Denotes whether persistency is enabled in the
  ///                     associated PARAGRAPH.
  ///
  /// This protected method is used by derived classes to implement their
  /// @p add_notifier methods.  The derived classes construct the appropriate
  /// @p invoker.
  ///
  /// Note that even if flow is previously set, an entry may be created to
  /// either (a) allow subsequent persistent PARAGRAPH invocations or
  /// (b) maintain notifier ordering if a notifier flush is in progress.
  //////////////////////////////////////////////////////////////////////
  template<typename NotifierInvoker>
  void add_notifier_impl(NotifierInvoker const& invoker,
                         Notifier const& notifier,
                         const bool b_persistent)
  {
    const bool b_notify_now = this->test_flow() && !this->test_flushing();

    if (!b_notify_now || b_persistent)
      m_notifications.push_back(*new notifier_entry_t(notifier));

    if (b_notify_now)
    {
      stapl_assert(
        this->empty_notifications() || b_persistent || this->test_flushing(),
        "flow set and notifications non empty");

      invoker(notifier);
    }

    // I'm not sure m_b_flow_set[req] can't have changed from above and haven't
    // thought through what that means.
    stapl_assert(b_notify_now == (this->test_flow() && !this->test_flushing()),
      "unexpected m_b_flow_set change in add_local_notifier");
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if there are no notifiers in list. False otherwise.
  //////////////////////////////////////////////////////////////////////
  bool empty_notifications(void) const
  {
    return m_notifications.empty() && !test_flushing();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Called by @p edge_entry destructor to explicit delete notifiers
  /// when they have been kept for PARAGRAPH persistency support.  For
  /// incrementally destroyed PARAGRAPHs, this is a noop.
  ///
  /// @sa edge_entry::~edge_entry
  //////////////////////////////////////////////////////////////////////
  void cleanup_notifications(void)
  {
    m_notifications.clear_and_dispose(notifier_entry_disposer());
  }
};  // class notifier_list

} // namespace detail

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_NOTIFIER_LIST_HPP

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_EDGE_VERSION_ENTRY_HPP
#define STAPL_PARAGRAPH_EDGE_VERSION_ENTRY_HPP

#include <boost/function.hpp>
#include <stapl/paragraph/edge_container/notifier_list.hpp>
#include <stapl/utility/tuple.hpp>

namespace stapl {

namespace detail {

template<typename T, typename Hook>
class filtered_edge_version_entry_base;


//////////////////////////////////////////////////////////////////////
/// @brief Define basic interface that all edge value versions should implement.
///   The stored type of the versions lists stored by @ref edge_entry.
/// @ingroup pgEdgeEntry
/// @tparam Hook Storage for container metadata.  Defined in Boost.Intrusive.
//////////////////////////////////////////////////////////////////////
template<typename Hook>
class edge_version_entry_base
  : public Hook
{
public:
  virtual ~edge_version_entry_base() = default;

  //////////////////////////////////////////////////////////////////////
  /// @brief Return true if the version has no notifications associated with it.
  //////////////////////////////////////////////////////////////////////
  virtual bool empty_notifications(void) const = 0;


  //////////////////////////////////////////////////////////////////////
  /// @brief Deconstruct notifier list in preparation for deletion of version.
  //////////////////////////////////////////////////////////////////////
  virtual void cleanup_notifications(void) = 0;


  //////////////////////////////////////////////////////////////////////
  /// @brief Reset data flow trigger prior to a persistent PARAGRAPH
  /// reinvocation.
  //////////////////////////////////////////////////////////////////////
  virtual void clear_flow(void) = 0;


  //////////////////////////////////////////////////////////////////////
  /// @brief Return request type (signal, filtered, full) of this version.
  //////////////////////////////////////////////////////////////////////
  virtual edge_request_type request(void) const = 0;
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object implementing the disposer concept defined by
///   Boost.Intrusive.
/// @ingroup pgEdgeEntry
///
/// Intrusive containers do not manage the lifetime of the objects they
/// contain.  This is left to the container's user to manage.  The disposer
/// callback functor allows the user to do this during container operations
/// where object deletion may be desired (i.e., clear()).  In the version
/// context, this disposer is a simple delete call.
///
/// Function operator signatures restricted to @p edge_version_entry_base and
/// @p filtered_edge_version_entry to guard against inadvertent use in other
/// contexts.
//////////////////////////////////////////////////////////////////////
struct version_entry_disposer
{
  template<typename Hook>
  void operator()(edge_version_entry_base<Hook>* entry_ptr) const
  {
    delete entry_ptr;
  }

  template<typename T, typename Hook>
  void operator()(filtered_edge_version_entry_base<T, Hook>* entry_ptr) const
  {
    delete entry_ptr;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Storage class for all edge flowed values that abstracts
/// how access to the value is provided.
///
/// Values are accessed in one of two ways:
///   (1) In object frame.  This occurs when object is flowed by copy or
///       move and the corresponding derived class version entry (and it's
///       edge entry) managed the lifetime of the object.
///
///   (2) Via an immutable shared wrapper to the value created by the runtime.
///       Used when sharing the value with another location in shared memory.
///
/// @todo Force initialization of storage by wrapper to receive an rvalue
/// for efficiency.
///
/// @todo Migration creates cases where we may legitimately call destructor
/// without ever initializing anything. If / when migration is pushed
/// through the typing system, assert in destructor
/// m_storage_state != uninitialized.
///
/// @todo Change @ref edge_container::migrate_entry so that constructor and
/// destructor can be protected access instead of public.
///
/// @todo Direct storage entries are not cleared during persistent execution
/// via clear_flow() but are left to be handled by the assignment operator in
/// set_value().  This releases memory back to the allocator in a more
/// predictable controlled manner, avoiding emprically measured performance
/// issues the allocator (i.e. pdt).  Need to insert logic to detect
/// when a value is dead during the execution traversal, to maintain same
/// incremental behavior while releasing memory as quickly as possible back
/// to the system.
//////////////////////////////////////////////////////////////////////
template<typename T>
class edge_version_storage
{
private:
  enum storage_state { uninitialized, direct_storage, wrapped, stolen };

  /// @brief Used to interpret state of storage union.
  storage_state                   m_storage_state;

  /// @brief Stores either the value or an immutable wrapper to the value.
  union {
    T                             m_value;
    immutable_shared<T>           m_wrapper;
  };

public:
  edge_version_storage(void)
    : m_storage_state(uninitialized)
  { }

  edge_version_storage(edge_version_storage const&)            = delete;
  edge_version_storage& operator=(edge_version_storage const&) = delete;

  ~edge_version_storage()
  {
    // migration can cause object to be destroyed without storage
    // ever being initialized.
    if (m_storage_state == direct_storage
        || m_storage_state == stolen)
    {
      m_value.~T();
      return;
    }

    if (m_storage_state == wrapped)
      m_wrapper.~immutable_shared<T>();
  }

protected:

  template<typename Q>
  void set_value(Q&& val)
  {
#if 0
    // see todo on class
    stapl_assert(m_storage_state == uninitialized,
      "initialized storage found");

    m_storage_state = direct_storage;
    new (&m_value) T(std::forward<Q>(val));
#endif

    stapl_assert(
      m_storage_state == uninitialized || m_storage_state == direct_storage,
       "initialized or indirect storage found");

    if (m_storage_state == uninitialized)
    {
      m_storage_state = direct_storage;
      new (&m_value) T(std::forward<Q>(val));
      return;
    }

    // else
    m_value = std::forward<Q>(val);
  }

  void set_value(immutable_shared<T>&& wrapper)
  {
    stapl_assert(m_storage_state == uninitialized,
      "initialized storage found");

    m_storage_state = wrapped;

    ::new (&m_wrapper) immutable_shared<T>(std::move(wrapper));
  }

  void set_value(immutable_shared<T> const& wrapper)
  {
    stapl_assert(m_storage_state == uninitialized,
      "initialized storage found");

    m_storage_state = wrapped;

    ::new (&m_wrapper) immutable_shared<T>(wrapper);
  }

public:
  bool is_direct_storage(void) const
  {
    stapl_assert(m_storage_state != uninitialized,
      "uninitialized storage found");

    return m_storage_state == direct_storage;
  }

  immutable_shared<T> wrapper(void) const
  {
    return m_wrapper;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Allows stored value to be stolen by a local consumer task
  /// if a runtime check (@ref edge_entry::stealable() conservatively
  /// detects the value is no longer needed.
  //////////////////////////////////////////////////////////////////////
  T&& steal(void)
  {
    stapl_assert(m_storage_state == direct_storage,
      "Attempting to steal without direct storage");

    m_storage_state = stolen;

    return std::move(m_value);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Reset state for reinvocation in a persistent paragraph.
  //////////////////////////////////////////////////////////////////////
  void clear_flow(void)
  {
    // only needed for version with notifiers
    // (e.g., signal with no signal consumers).
    if (m_storage_state == uninitialized
        || m_storage_state == direct_storage)
      return;

    if (m_storage_state == stolen)
      m_value.~T();
    else
      m_wrapper.~immutable_shared<T>();

    m_storage_state = uninitialized;
  }

  T const& value(void) const
  {
    stapl_assert(m_storage_state != uninitialized,
      "uninitialized storage found");

    stapl_assert(m_storage_state != stolen,
      "stolen storage found");

    if (m_storage_state == direct_storage)
      return m_value;

    // else
    return m_wrapper.get();
  }
}; // class edge_version_storage


//////////////////////////////////////////////////////////////////////
/// @brief Specialization for signal / void edge flow.  Maintain
/// convention of treating it internally as an int.
//////////////////////////////////////////////////////////////////////
template<>
class edge_version_storage<void>
  : public edge_version_storage<int>
{ };


//////////////////////////////////////////////////////////////////////
/// @brief Notifier invoker passed by @ref edge_version_entry
/// to @ref notifier_list base class.
///
/// Explicit functor instead of lambda to reduce compilation pressure
/// and code dupe (multiple usage sites).
//////////////////////////////////////////////////////////////////////
template<typename Notifier, typename T>
class notifier_invoker
{
private:
  executor_base*           m_executor_ptr;
  edge_version_storage<T>& m_storage;

public:
  notifier_invoker(executor_base* executor_ptr,
                   edge_version_storage<T>& storage)
    : m_executor_ptr(executor_ptr),
      m_storage(storage)
  { }

  void operator()(Notifier const& notifier) const
  {
    stapl_assert(m_executor_ptr != nullptr, "null executor pointer");

    notifier(*m_executor_ptr, m_storage.value());
  }
}; // class notifier_invoker


//////////////////////////////////////////////////////////////////////
/// @brief Version notifier list used for FULL value edge consumption.
/// @ingroup pgEdgeEntry
///
/// @tparam T The edge value type that is flowed to successors.
/// @tparam Notifier The type of notifiers this list stores.
/// @tparam Hook @tparam Hook Storage for container metadata.
///   Defined in Boost.Intrusive. Passed to base class.
///   @p edge_version_entry_base.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Notifier, typename Hook>
class edge_version_entry
  : public notifier_list<Notifier>,
    public edge_version_entry_base<Hook>,
    public edge_version_storage<T>
{
public:
  STAPL_USE_MANAGED_ALLOC(edge_version_entry)

  edge_version_entry(void) = default;

  edge_version_storage<T> const& version_storage(void) const
  {
    return *this;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add a new notifier to the list, possibly invoking it immediately
  ///   if this version of the edge value is already available.
  /// @param notifier The edge notifier that is tied to the data flow this
  ///   notifier list is a part of.
  /// @param executor The executor of the PARAGRAPH this edge version is
  ///   associated with.
  /// @param b_persistent Denotes whether persistency is enabled in the
  ///    associated PARAGRAPH.
  ///
  /// Constructs an invoker function object which abstracts the signature of
  ///   the notifier before calling generic implementation in
  ///   @ref notifier_list::add_notifier_impl.
  //////////////////////////////////////////////////////////////////////
  edge_version_storage<T>&
  add_notifier(Notifier const& notifier,
               executor_base& executor,
               const bool b_persistent)
  {
    this->add_notifier_impl(
      notifier_invoker<Notifier, T>(&executor, *this), notifier, b_persistent);

    return *this;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Receive data flow of version's edge value and invoke any
  ///   registered notifiers.
  /// @param val The value from the producer task to this version.
  /// @param executor_ptr The executor of the PARAGRAPH this edge version is
  ///   associated with.
  /// @param b_migration_enabled Denotes whether task migration is enabled in
  ///   the associated PARAGRAPH.
  /// @param b_persistent Denotes whether persistency is enabled in the
  ///    associated PARAGRAPH.
  ///
  /// Stores the @p val in @p m_value and calls
  /// @ref notifier_list::flush_notifications passing it notifier invoker
  /// to abstract the notifier's calling signature.
  /// @todo Enable migrated edge value assertion.
  //////////////////////////////////////////////////////////////////////
  template<typename Q>
  void set_value(Q&& val,
                 executor_base* executor_ptr,
                 const bool b_migration_enabled,
                 const bool b_persistent)
  {
    if (b_migration_enabled && this->test_flow())
    {
      // stapl_assert(this->value() == val,
      //   "value previously set does not equal new value");
      return;
    }

    stapl_assert(!this->test_flow(), "flow already set");

    edge_version_storage<T>::set_value(std::forward<Q>(val));

    this->flush_notifications(
      notifier_invoker<Notifier, T>(executor_ptr, *this), b_persistent);

    this->set_flow();
  }

  /////////////////////////////////////////////////////////////////////////
  /// @brief Implementation of pure virtual method in abstract base
  /// @ref edge_version_entry_base.
  ///
  /// @sa edge_version_entry_base::empty_notifications
  //////////////////////////////////////////////////////////////////////
  bool empty_notifications(void) const final
  {
    return notifier_list<Notifier>::empty_notifications();
  }


  /////////////////////////////////////////////////////////////////////////
  /// @brief Implementation of pure virtual method in abstract base
  /// @ref edge_version_entry_base.
  ///
  /// @sa edge_version_entry_base::cleanup_notifications
  //////////////////////////////////////////////////////////////////////
  void cleanup_notifications(void) final
  {
    notifier_list<Notifier>::cleanup_notifications();
  }


  /////////////////////////////////////////////////////////////////////////
  /// @brief Implementation of pure virtual method in abstract base
  /// @ref edge_version_entry_base.
  ///
  /// @sa edge_version_entry_base::cleanup_notifications
  //////////////////////////////////////////////////////////////////////
  void clear_flow(void) final
  {
    notifier_list<Notifier>::clear_flow();
    edge_version_storage<T>::clear_flow();
  }


  /////////////////////////////////////////////////////////////////////////
  /// @brief Implementation of pure virtual method in abstract base
  /// @p edge_version_entry_base.
  ///
  /// @sa edge_version_entry_base::request
  //////////////////////////////////////////////////////////////////////
  edge_request_type request(void) const final
  {
    return FULL;
  }
}; // class edge_version_entry


//////////////////////////////////////////////////////////////////////
/// @brief Notifier invoker passed by @ref edge_version_entry
/// (void value specialization) to @ref notifier_list base class.
///
/// Explicit functor instead of lambda to reduce compilation pressure
/// and code dupe (multiple usage sites).
//////////////////////////////////////////////////////////////////////
template<typename Notifier>
class notifier_void_invoker
{
private:
  executor_base* m_executor_ptr;

public:
  notifier_void_invoker(executor_base* executor_ptr)
    : m_executor_ptr(executor_ptr)
  { }

  void operator()(Notifier const& notifier) const
  {
    stapl_assert(m_executor_ptr != nullptr, "null executor pointer");

    notifier(*m_executor_ptr);
  }
}; // class notifier_void_invoker


//////////////////////////////////////////////////////////////////////
/// @brief Specialization of basic version notifier list for signals.
/// @ingroup pgEdgeEntry
///
/// @tparam Hook @tparam Hook Storage for container metadata.
///   Defined in Boost.Intrusive. Passed to base class.
///
/// In this case there is no consumed value. The edge value type is logically
/// void. Hence there is no need for a member variable to store the flowed
/// value.  Since signals are stored in the frame of @p edge_entry_base,
/// This class template specialization does not inherit from
/// @p edge_version_entry_base and need not define interface of that abstract
/// class.
//////////////////////////////////////////////////////////////////////
template<typename Notifier, typename Hook>
class edge_version_entry<void, Notifier, Hook>
  : public notifier_list<Notifier>,
    public Hook,
    public edge_version_storage<void>
{
public:
  STAPL_USE_MANAGED_ALLOC(edge_version_entry)

  edge_version_entry(void) = default;

  edge_version_entry(edge_version_entry&& other)
    : notifier_list<Notifier>(std::move(other))
  { }

  edge_version_storage<void> const& version_storage(void) const
  {
    return *this;
  }

  void clear_flow(void)
  {
    notifier_list<Notifier>::clear_flow();
    edge_version_storage<void>::clear_flow();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add a new notifier to the list, possibly invoking it
  ///   immediately if this version of the edge value is already available.
  /// @param notifier The edge notifier that is tied to the data flow this
  ///   notifier list is a part of.
  /// @param executor The executor of the PARAGRAPH this edge version is
  ///   associated with.
  /// @param b_persistent Denotes whether persistency is enabled in the
  ///    associated PARAGRAPH.
  ///
  /// Constructs an invoker function object which abstracts the signature
  ///   of the notifier before calling generic implementation in
  ///   @ref notifier_list::add_notifier_impl.
  //////////////////////////////////////////////////////////////////////
  void add_notifier(Notifier const& notifier,
                    executor_base& executor,
                    const bool b_persistent)
  {
    this->add_notifier_impl(
      notifier_void_invoker<Notifier>(&executor), notifier, b_persistent);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive signal flow of predecessor task and invoke any
  ///   registered notifiers.
  /// @param executor_ptr The executor of the PARAGRAPH this edge version is
  ///   associated with.
  /// @param b_migration_enabled Denotes whether task migration is enabled
  ///   in the associated PARAGRAPH.
  /// @param b_persistent Denotes whether persistency is enabled in the
  ///    associated PARAGRAPH.
  ///
  /// Calls @ref notifier_list::flush_notifications passing it notifier
  ///   invoker to abstract the notifier's calling signature.
  //////////////////////////////////////////////////////////////////////
  void set_value(executor_base* executor_ptr,
                 const bool b_migration_enabled,
                 const bool b_persistent)
  {
    stapl_assert(!this->test_flow() || b_migration_enabled,
      "signal previous set");

    if (this->test_flow())
      return;

    this->flush_notifications(
      notifier_void_invoker<Notifier>(executor_ptr), b_persistent);

    this->set_flow();
  }
}; // class edge_version_entry<void>


//////////////////////////////////////////////////////////////////////
/// @brief abstract base class template of @ref filtered_edge_version_entry
/// used to store entries in @ref filtered_edge_versions.  Allow state
/// querying and full value based edge flow to be performed without having
/// to know the type of the filtered edge.
///
/// @tparam FullEdge Type of value created by the producer task.
/// @tparam Hook The container metadata for Boost.Intrusive.
//////////////////////////////////////////////////////////////////////
template<typename FullEdge, typename Hook>
class filtered_edge_version_entry_base
  : public Hook
{
public:
  virtual ~filtered_edge_version_entry_base()  = default;
  virtual bool empty_notifications(void) const = 0;
  virtual void cleanup_notifications(void)     = 0;
  virtual void clear_flow(void)                = 0;
  virtual bool test_flow(void) const           = 0;
  virtual void set_value_full(FullEdge const&, executor_base*, bool, bool) = 0;
}; // class filtered_edge_version_entry_base


//////////////////////////////////////////////////////////////////////
/// @brief Derived class of filtered version which stored Filter and
/// implemented methods requiring this type.
///
/// @tparam FullEdge Type of value created by the producer task.
/// @tparam Filter The filter applied prior to forwarding value to consumer.
/// @tparam Hook The container metadata for Boost.Intrusive.
//////////////////////////////////////////////////////////////////////
template<typename FullEdge, typename Filter, typename Hook>
class filtered_edge_version_entry final
  : public filtered_edge_version_entry_base<FullEdge, Hook>,
    public notifier_list<
      boost::function<
        void (executor_base&,
              typename boost::result_of<Filter(FullEdge)>::type const&)>
    >,
    public edge_version_storage<
      typename boost::result_of<Filter(FullEdge)>::type>,
    private Filter
{
private:
  typedef typename boost::result_of<
    Filter(FullEdge)>::type                         filtered_edge_t;

  typedef boost::function<
    void (executor_base&, filtered_edge_t const&)>  notifier_t;

  typedef notifier_list<notifier_t>                 base_t;

public:
  STAPL_USE_MANAGED_ALLOC(filtered_edge_version_entry)

  filtered_edge_version_entry(Filter const& filter)
    : Filter(filter)
  { }

  bool empty_notifications(void) const
  { return notifier_list<notifier_t>::empty_notifications(); }

  void cleanup_notifications(void)
  { notifier_list<notifier_t>::cleanup_notifications(); }

  bool test_flow(void) const
  { return notifier_list<notifier_t>::test_flow(); }

  void clear_flow(void)
  {
    notifier_list<notifier_t>::clear_flow();
    edge_version_storage<filtered_edge_t>::clear_flow();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Receive an already filtered value and trigger data flow on the
  /// associated filtered version.
  ///
  /// @param val The filtered edge value.
  /// @param executor_ptr The executor of the PARAGRAPH this edge version is
  ///   associated with.
  /// @param b_migration_enabled Denotes whether task migration is enabled in
  ///   the associated PARAGRAPH.
  /// @param b_persistent Denotes whether persistency is enabled in the
  ///    associated PARAGRAPH.
  //////////////////////////////////////////////////////////////////////
  template<typename ValueParam>
  void set_value(ValueParam&& val, executor_base* executor_ptr,
                 bool b_migration_enabled, bool b_persistent)
  {
    // Migration not enabled for filtering yet.  Therefore flow should
    // never be set on a set_value() call.
    stapl_assert(!this->test_flow(), "found filtered value set");

    edge_version_storage<filtered_edge_t>::set_value(
      std::forward<ValueParam>(val));

    this->flush_notifications(
      notifier_invoker<notifier_t, filtered_edge_t>(executor_ptr, *this),
      b_persistent);

    this->set_flow();
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Receive full data flow of producer task, apply the version's filter
  ///   before storing value and invoking any registered notifiers.
  ///
  /// @param val The full edge value from the producer task to this version.
  /// @param executor_ptr The executor of the PARAGRAPH this edge version is
  ///   associated with.
  /// @param b_migration_enabled Denotes whether task migration is enabled in
  ///   the associated PARAGRAPH.
  /// @param b_persistent Denotes whether persistency is enabled in the
  ///    associated PARAGRAPH.
  ///
  /// In the presence of full consumers on the location where this filtered
  /// version list exists (i.e., "covering" this version), it is possible that
  /// the protocol will avoid applying the filter on the producer location
  /// (since it has to transport the full value anyways).  In this case, the
  /// @p edge_entry will invoke this method with the full value available to it.
  ///
  /// @sa edge_version_entry::set_value
  //////////////////////////////////////////////////////////////////////
  void set_value_full(FullEdge const& val, executor_base* executor_ptr,
                      bool b_migration_enabled, bool b_persistent) final
  {
    // a previous remote FILTER notification req was serviced by producer and
    // has fired FILTERED clients.
    if (this->test_flow())
    {
       // Just assert we get the same filtered value.
       // need EquivalenceComparable value type (proxy_holder fails right now)
 #if 0
       stapl_assert(m_filter_func(val) == this->value(FILTERED),
         "filtered full / m_filter_func values differ");
 #endif

       stapl_assert(b_persistent || this->empty_notifications(),
         "flow_set with pending notifications");

       return;
     }

     // else, the remote FILTERED req was upgraded to FULL.
     this->set_value(static_cast<Filter&>(*this)(val),
                     executor_ptr, b_migration_enabled, b_persistent);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Add a new notifier to the list, possibly invoking it immediately
  /// if this version of the edge value is already available.
  ///
  /// @param notifier The edge notifier that is tied to the data flow this
  ///   notifier list is a part of.
  /// @param filter The filter to be applied to a full edge flow.
  /// @param executor The executor of the PARAGRAPH this edge version is
  ///   associated with.
  /// @param full_value Reference to full edge value storage in the
  ///    @p edge_entry (may not be initialized).
  /// @param b_full_flow Denotes whether @p full_value is initialized and can
  ///   be used to computer filtered value for this version.
  /// @param b_migration_enabled Denotes whether task migration is enabled in
  ///   the associated PARAGRAPH.
  /// @param b_persistent Denotes whether persistency is enabled in the
  ///    associated PARAGRAPH.
  ///
  /// Notifiers for filtered versions can be serviced by the full edge value if
  /// it's already available on this location (in addition to a previously
  /// flowed copy of the same filtered value).  When adding a new notifier,
  /// check if the full value is available and initialize @p m_value if it is
  /// and this versions flow has not been previously set.  Then, the base
  /// class implementation of @p add_notifier is called.
  ///
  /// @sa edge_version_entry::add_notifier
  //////////////////////////////////////////////////////////////////////
  template<typename FullValueAccess>
  edge_version_storage<filtered_edge_t>&
  add_notifier(notifier_t const& notifier,
               Filter const& filter,
               executor_base& executor,
               FullValueAccess const& full_value,
               bool b_full_flow,
               bool b_migration_enabled,
               bool b_persistent)
  {
    if (!this->test_flow() && b_full_flow)
    {
      this->set_value(static_cast<Filter&>(*this)(full_value()), &executor,
                      b_migration_enabled, b_persistent);
    }

    this->add_notifier_impl(
      notifier_invoker<notifier_t, filtered_edge_t>(&executor, *this),
      notifier, b_persistent);

    return *this;
  }
}; // class filtered_edge_version_entry_base


//////////////////////////////////////////////////////////////////////
/// @brief Represents a list of versions, each of which represents a filtered
///   edge consumption with a different filter.
/// @ingroup pgEdgeEntry
///
/// @todo separating out the dependence tree (for lack of a better phrase) or
/// the lattice of partial edge relationships from the addition of a notifier
/// would unify this with the other add_notifier signature.
/// In edge_entry::add_local_notifier...
/// (1) initialize version entry with filter
/// (2) setup relation
/// (3) propagate flow if possible (otherwise relation will trigger it later.
/// (4) then call this entry's add_notifier() method.
///
/// @todo Can void needless memory allocation in @p add_notifier with some
/// full_flow / persistent logic..
///
/// @todo It may be more advantageous to add to the front of the version list
/// (temporal locality).  However, if we leave it as is (adding to back) and
/// still search for matching filters, no need to cache_last as we arrive at
/// end after exhaustive search.
//////////////////////////////////////////////////////////////////////
template<typename FullEdge, typename Hook>
class filtered_edge_versions final
  : public edge_version_entry_base<Hook>
{
private:
  typedef boost::intrusive::slist_base_hook<>         entry_hook_t;

  /// @brief The entry type of a single filtered version stored the filtered
  /// versions list.
  typedef filtered_edge_version_entry_base<
    FullEdge, entry_hook_t>                           filtered_entry_t;

  /// @brief The list type used to store filtered versions.
  typedef boost::intrusive::slist<
    filtered_entry_t,
    boost::intrusive::constant_time_size<false>,
    boost::intrusive::cache_last<true>>               filtered_list_t;

  /// @brief The list of filtered versions managed by the class.
  filtered_list_t                                     m_versions;


  //////////////////////////////////////////////////////////////////////
  /// @brief return iterator to version entry associated with specified
  /// version identifier. Not const qualified.
  //////////////////////////////////////////////////////////////////////
  typename filtered_list_t::iterator
  get_version_iter(unsigned int version_id)
  {
    unsigned int cnt = 0;

    --version_id;

    auto iter = m_versions.begin();

    for (; cnt < version_id && iter != m_versions.end(); ++cnt, ++iter)
    { }

    return iter;
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief return iterator to version entry associated with specified
  /// version identifier. const qualified.
  //////////////////////////////////////////////////////////////////////
  typename filtered_list_t::const_iterator
  get_version_iter(unsigned int version_id) const
  {
    unsigned int cnt = 0;

    --version_id;

    auto iter = m_versions.begin();

    for (; cnt < version_id && iter != m_versions.end(); ++cnt, ++iter)
    { }

    return iter;
  }


public:
  STAPL_USE_MANAGED_ALLOC(filtered_edge_versions)

  filtered_edge_versions(void) = default;

  ~filtered_edge_versions() final
  {
    m_versions.clear_and_dispose(version_entry_disposer());
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Receive an already filtered value and trigger data flow on the
  /// associated filtered version.
  ///
  /// @param val The filtered edge value.
  /// @param index The position of the filtered version in the version list
  ///   @p m_versions.
  /// @param executor_ptr The executor of the PARAGRAPH this edge version is
  ///   associated with.
  /// @param b_migration_enabled Denotes whether task migration is enabled in
  ///   the associated PARAGRAPH.
  /// @param b_persistent Denotes whether persistency is enabled in the
  ///    associated PARAGRAPH.
  /// @param b_expect_exist Denotes whether @ref edge_entry expects a version
  ///   to exist when data flow is trigger.  This is true or filtered versions.
  ///   The parameter is unused but included to match @p set_value signature
  ///    included to match @p set_value signature
  ///
  /// @p index was previously returned to the @p edge_entry from @p add_notifier
  /// and used create a remote notifier that was sent to the producer task's
  /// execution location.  The invocation of that remote notifier triggered
  /// the callback to this method through the local @ref edge_entry.
  //////////////////////////////////////////////////////////////////////
  template<typename Filter, typename ValueParam>
  void set_value(ValueParam&& val, unsigned int index,
                 executor_base* executor_ptr,
                 bool b_migration_enabled, bool b_persistent,
                 bool b_expect_exist)
  {
    auto iter = get_version_iter(index);

    // without migration support for filtering consumption, I expect an entry
    // to always exist when set_value() is called right now.
    stapl_assert(iter != m_versions.end(), "no entry found");

    typedef filtered_edge_version_entry<
      FullEdge, Filter, entry_hook_t> filtered_entry_t;

    down_cast<filtered_entry_t>(*iter).set_value(
      std::forward<ValueParam>(val), executor_ptr,
      b_migration_enabled, b_persistent);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Add a new notifier for version defined by @p filter, creating a
  /// new version and inserting it the versions list if no local consumers using
  /// the filter were previously created.  Notifier may be immediately invoked
  /// if the version of edge value is already available.
  ///
  /// @param notifier The edge notifier that is tied to the data flow this
  ///   notifier list is a part of.
  /// @param filter The filter to be applied to a full edge flow.
  /// @param executor The executor of the PARAGRAPH this edge version is
  ///   associated with.
  /// @param full_value Reference to full edge value storage in the
  ///    @p edge_entry (may not be initialized).
  /// @param b_full_flow Denotes whether @p full_value is initialized and can
  ///   be used to compute filtered value for this version.
  /// @param b_migration_enabled Denotes whether task migration is enabled in
  ///   the associated PARAGRAPH.
  /// @param b_persistent Denotes whether persistency is enabled in the
  ///    associated PARAGRAPH.
  ///
  /// @return A four element tuple used by caller
  ///   (@ref edge_entry::add_local_notifier) to determine if and how a
  ///    inter-location remote identifier needs to be created Specifically:
  ///      1: Whether flow for the version has previously been set.
  ///      2: The index (starting at 1) of the version in the versions list.
  ///      3: Whether a previous consumer had registered a notifier for this
  ///         version (i.e., is consumption "covered").
  ///      4: A reference to the version storage (used to initialize wf
  ///         reference to this value).
  /// @sa filtered_edge_version_entry::add_notifier
  /// @todo Receive filter parameter by universal reference.
  ///
  /// @todo An Attempt to aggregate requests with the same filter is disabled
  /// below, due to lack of use in encountered cases and the additional
  /// runtime overhead to do so.  Maybe add an assert in debug mode to
  /// to detect when aggregating would have succeeded.
  //////////////////////////////////////////////////////////////////////
  template<typename Filter, typename FullValueAccess>
  tuple<bool, unsigned int, bool,
        edge_version_storage<
          typename boost::result_of<Filter(FullEdge)>::type>&>
  add_notifier(boost::function<
                 void (executor_base&,
                       typename boost::result_of<
                         Filter(FullEdge)>::type const&)
               > const& notifier,
               Filter const& filter,
               executor_base& executor,
               FullValueAccess const& full_value,
               bool b_full_flow,
               bool b_migration_enabled,
               bool b_persistent)
  {
    typedef tuple<bool, unsigned int, bool,
                  edge_version_storage<
                    typename boost::result_of<
                      Filter(FullEdge)>::type>&> return_t;
    size_t idx = 0;

    for (auto iter = m_versions.begin(); iter != m_versions.end(); ++iter)
      ++idx;

#if 0
    for (auto&& version_entry : m_versions)
    {
      if (version_entry.filter_equal(filter))
      {
        edge_version_storage<T> const& storage_ref =
          version_entry.add_notifier(
            notifier, filter, executor, full_value,
            b_full_flow, b_migration_enabled, b_persistent);

        return return_t(version_entry.test_flow(), idx, false, storage_ref);
      }
      ++idx;
    }
#endif

    using filtered_entry_t =
      filtered_edge_version_entry<FullEdge, Filter, entry_hook_t>;

    // no match found, create new entry.
    auto entry_ptr = new filtered_entry_t(filter);

    m_versions.push_back(*entry_ptr);

    auto& storage_ref =
      entry_ptr->add_notifier(notifier, filter, executor, full_value,
                              b_full_flow, b_migration_enabled, b_persistent);

    return return_t(entry_ptr->test_flow(), idx, true, storage_ref);
  }


  //////////////////////////////////////////////////////////////////////
  /// @brief Receive full data flow of producer task and pass it to all
  ///   filtered versions to trigger data flow.
  ///
  /// @param val The full edge value from the producer task to this version.
  /// @param executor_ptr The executor of the PARAGRAPH this edge version is
  ///   associated with.
  /// @param b_migration_enabled Denotes whether task migration is enabled in
  ///   the associated PARAGRAPH.
  /// @param b_persistent Denotes whether persistency is enabled in the
  ///    associated PARAGRAPH.
  ///
  /// @sa filtered_edge_version_entry::set_value_full
  //////////////////////////////////////////////////////////////////////
  void set_value_full(FullEdge const& val,
                      executor_base* executor_ptr,
                      bool b_migration_enabled,
                      bool b_persistent)
  {
    for (auto&& version_entry : m_versions)
      version_entry.set_value_full(
        val, executor_ptr, b_migration_enabled, b_persistent);
  }


  /////////////////////////////////////////////////////////////////////////
  /// @brief Implementation of pure virtual method in abstract base
  /// @p edge_version_entry_base.
  ///
  /// @return True if all filtered versions have empty notification lists.
  ///
  /// @sa edge_version_entry_base::empty_notifications
  //////////////////////////////////////////////////////////////////////
  bool empty_notifications(void) const final
  {
    for (auto&& version_entry : m_versions)
      if (!version_entry.empty_notifications())
        return false;

    return true;
  }


  /////////////////////////////////////////////////////////////////////////
  /// @brief Implementation of pure virtual method in abstract base
  /// @p edge_version_entry_base.
  ///
  /// Clear notifications of all filtered versions in @p m_versions.
  ///
  /// @sa edge_version_entry_base::cleanup_notifications
  //////////////////////////////////////////////////////////////////////
  void cleanup_notifications(void) final
  {
    for (auto&& version_entry : m_versions)
      version_entry.cleanup_notifications();
  }


  /////////////////////////////////////////////////////////////////////////
  /// @brief Implementation of pure virtual method in abstract base
  /// @p edge_version_entry_base.
  ///
  /// @sa edge_version_entry_base::request
  //////////////////////////////////////////////////////////////////////
  edge_request_type request(void) const final
  {
    return FILTERED;
  }


  /////////////////////////////////////////////////////////////////////////
  /// @brief Implementation of pure virtual method in abstract base
  /// @p edge_version_entry_base.
  ///
  /// Clear flow bit of all filtered versions in @p m_versions.
  ///
  /// @sa edge_version_entry_base::clear_flow
  //////////////////////////////////////////////////////////////////////
  void clear_flow(void) final
  {
    for (auto&& version_entry : m_versions)
      version_entry.clear_flow();
  }


  /////////////////////////////////////////////////////////////////////////
  /// @brief Returns true if data flow has previously been triggered for
  /// filtered version associated with @p version_id;
  //////////////////////////////////////////////////////////////////////
  bool test_flow(unsigned int version_id) const
  {
    auto iter = this->get_version_iter(version_id);

    return iter == m_versions.end() ? false : iter->test_flow();
  }
}; // class filtered_edge_versions

} // namespace detail

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_EDGE_VERSION_ENTRY_HPP



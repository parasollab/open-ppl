/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_PARAGRAPH_EDGE_CONTAINER_HPP
#define STAPL_PARAGRAPH_EDGE_CONTAINER_HPP

#include <stapl/paragraph/edge_container/edge_container.h>
#include <stapl/paragraph/edge_container/edge_remote_notifier.hpp>
#include <stapl/paragraph/edge_container/edge_notifier_wrapper.hpp>
#include <stapl/paragraph/edge_container/utility.hpp>
#include <stapl/paragraph/edge_container/edge_entry.hpp>

namespace stapl {

inline
edge_container::~edge_container()
{
  stapl_assert(m_unknown_consumer_cnt == 0,
    "~edge_container() found non-zero m_unknown_consumer_cnt");

  stapl_assert(m_deferred_specs == 0,
    "~edge_container() found non-zero m_deferred_specs");

  stapl_assert(this->is_persistent() || this->empty(),
      "~edge_container: cache has entries");

  if (!this->is_persistent())
    return;

  // For Persistent PGs, cleanup of entries has been deferred until now.
  //
  // (1) Iterate over all entries, clearing notification lists.  This should
  //     move all entries to an evictable state.
  //
  // (2) Clear container, calling disposer on each element to delete
  //     intrusive element.
  this->for_each([](edge_entry_t& entry) { entry.cleanup_notifications(); });

  this->clear();
}


inline
void edge_container::receive_signal(const index_type tid)
{
  edge_entry_t& entry = this->lookup_expect(tid);

  this->start_hold();

  entry.set_signal(this->tg().executor_ptr(), this->migration_enabled());

  this->try_base_eviction(entry);

  this->stop_hold();
}


inline
bool edge_container::migration_enabled(void) const
{
  return m_tg_ptr->migration_enabled();
}


template<typename T, typename Filter>
tuple<detail::edge_entry_base*,
      detail::edge_version_storage<
         typename df_stored_type<
           typename boost::result_of<
             Filter(typename df_stored_type<T>::type)>::type
         >::type>*>
edge_container::
setup_flow(index_type producer_tid,
           boost::function<
             void (executor_base&,
                   typename df_stored_type<
                     typename boost::result_of<
                       Filter(typename df_stored_type<T>::type)>::type
                   >::type const&)
           > const& notifier,
           Filter const& filter,
           bool b_inc_local_usr_cnt)
{
  typedef detail::edge_entry<T>                         derived_edge_entry_t;

  typedef tuple<
     detail::edge_entry_base*,
     detail::edge_version_storage<
       typename df_stored_type<
         typename boost::result_of<
           Filter(typename df_stored_type<T>::type)
         >::type>::type>*>                             return_t;

  edge_entry_t* entry_ptr = this->lookup_ptr(producer_tid);

  // Does this location already know something about producer tid?
  if (entry_ptr != nullptr)
  {
    // Verify that entry is fully baked (i.e., value holding) entry,
    // not some wimpy "signal only" base entry.  If it's not,
    // make it so.
    if (entry_ptr->is_signal_only())
    {
      edge_entry_t& base_entry_ref = *entry_ptr;

      // I know this looks weird, but trust me, it's not leaked.
      // base_entry_ref is in a container, and it's hooks are
      // moved into the newly created entry.
      entry_ptr = new derived_edge_entry_t(std::move(base_entry_ref));

      delete &base_entry_ref;
    }

    // Register myself as a local consumer. Return value from entry call
    // gives us some information about the previous state of entry and
    // directs our actions below.
    //
    derived_edge_entry_t& entry_ref = get_entry_ref<T>(*entry_ptr);

    // NOTE - this check is only needed when migration is enabled.  Guard if
    // we choose to ever type guard migration support throughout the PARAGRAPH.
    //
    // Helps us set the upgrade bit properly for remote notifier creation
    // when we have a local producer already initialized.  In this case, the
    // presence of an entry doesn't mean this is an upgrade.  If there's no
    // previous consumer, the upgrade bit for send_remote_notifier() should be
    // false.  Grab the previous state before inserting this local consumer.
    const bool b_had_previous_consumer =
      entry_ref.has_unnotified_local_consumer();

    const auto ret_val =
      entry_ref.add_local_notifier(
        *this, notifier, b_inc_local_usr_cnt, filter);

    // This loc hasn't been assigned to exec producer_tid (still possible it
    // might), but has another consumer previously created who request covers
    // the current request (e.g., this is SIGNAL and previously a FULL was
    // requested).
    //
    // Send a message to the producer that he should decrement
    // his successor count but does not need to send the value (as send has
    // already been requested).
    if (get<0>(ret_val) == REMOTE_REQ_COVERED)
    {
      decrement_unknown_consumers(*entry_ptr);

      return return_t(&entry_ref, &get<2>(ret_val));
    }

    // else (get<0>(ret_val) == SEND_NOTIFIER_REQ)
    //
    // The producer is local and a local consumer is requesting its value
    // but no other consumer so far has requested the value. In this case
    // request addition of a remote notifier locally without setting the
    // upgrade bit.
    //
    // Doing this, simplifies the task migration protocol.
    if (entry_ptr->local_producer_initialized())
    {
      if (this->migration_enabled()) {
        send_remote_notifier<T>(
          producer_tid, &entry_ref, filter,
          get<1>(ret_val), b_had_previous_consumer, true);

        return return_t(&entry_ref, &get<2>(ret_val));
      }

      // else
      decrement_unknown_consumers(*entry_ptr);

      return return_t(&entry_ref, &get<2>(ret_val));
    }

    // Now we are sure that in this if condition, the upgrade bit has to be set
    // because we can be here if and only if:
    //
    // 1. There was a local entry for the task.
    // 2. Producer is not here (not init'd or created / migrated elsewhere).
    // 3. A consumer has previously requested a value which does not cover
    //    this request
    send_remote_notifier<T>(
      producer_tid, &entry_ref, filter, get<1>(ret_val), true);

    return return_t(&entry_ref, &get<2>(ret_val));
  }

  // Neither the producer nor another consumer init'd yet at this location.
  //
  // (1) Create a new entry for producer_tid in the local cache.
  derived_edge_entry_t& entry_ref =
    *new derived_edge_entry_t(producer_tid, this->is_persistent());

#ifndef STAPL_NDEBUG
  bool b_insert_success =
#endif
    this->insert(entry_ref);

  stapl_assert(b_insert_success, "Insert failed edge_container::setup_flow");

  // (2) Register myself as a local consumer.
  const auto ret_val =
    entry_ref.add_local_notifier(*this, notifier, b_inc_local_usr_cnt, filter);

  stapl_assert(get<0>(ret_val) == SEND_NOTIFIER_REQ, "!SEND_NOTIFIER_REQ");

  // (3) Fire of a notifier with upgrade bit set to false.
  send_remote_notifier<T>(
    producer_tid, &entry_ref, filter, get<1>(ret_val), false);

  return return_t(&entry_ref, &get<2>(ret_val));
}


inline
void edge_container::setup_signal_flow(index_type producer_tid,
                                       signal_notifier_t const& notifier)
{
  edge_entry_t* entry_ptr = this->lookup_ptr(producer_tid);

  // Does this location already know something about producer tid?
  if (entry_ptr != nullptr)
  {
    // Register myself as a local consumer. Return value from entry call
    // gives us some information about the previous state of entry and
    // directs our actions below.
    const df_add_status ret_val =
      entry_ptr->add_signal_notifier(this->tg().executor(), notifier);

    // This loc hasn't been assigned to exec producer_tid (still possible it
    // might), but has another consumer previously created who request covers
    // the current request (e.g., this is SIGNAL and previously a FULL was
    // requested).
    //
    // Send a message to the producer that he should decrement
    // his successor count but does not need to send the value (as send has
    // already been requested).
    if (ret_val == REMOTE_REQ_COVERED)
    {
      decrement_unknown_consumers(*entry_ptr);

      return;
    }

    // else (ret_val == SEND_NOTIFIER_REQ)
    //
    // This is a local producer and a local consumer is requesting its value
    // but no other consumer so far has requested the value. In this case
    // request addition of a remote notifier locally without setting the
    // upgrade bit.
    // Doing this, simplifies the task migration protocol.
    if (entry_ptr->local_producer_initialized())
    {
      // Upgrade bit is false.  If adding a signal consumer returned
      // SEND_NOTIFIER_REQ, then this is the first consumer and hence
      // is not an upgrade.
      send_remote_signal_notifier(producer_tid, false, true);

      return;
    }

    stapl_assert(0, "entry found, not producer, upgrade requested for signal");

    return;
  }

  // Neither the producer nor another consumer init'd yet at this location.
  //
  // (1) Create a new entry for producer_tid in the local cache.
  edge_entry_t& entry_ref =
    *new edge_entry_t(producer_tid, this->is_persistent());

#ifndef STAPL_NDEBUG
  bool b_insert_success =
#endif
    this->insert(entry_ref);

  stapl_assert(b_insert_success,
               "Insert failed edge_container_bas::setup_signal_flow");

  // (2) Register myself as a local consumer.
#ifndef STAPL_NDEBUG
  const df_add_status ret_val =
#endif
    entry_ref.add_signal_notifier(this->tg().executor(), notifier);

  stapl_assert(ret_val == SEND_NOTIFIER_REQ, "expected SEND_NOTIFIER_REQ");

  // (3) Fire of a notifier with upgrade bit set to false.
  send_remote_signal_notifier(producer_tid, false);
}


template<typename T, typename ValueParam>
void edge_container::
nested_sibling_set_element(const index_type tid, ValueParam&& val)
{
  // If this is the first time my producer sibling has flowed a value to me,
  // call add_producer to set up the appropriate @ref edge_entry.
  if (!tg().called_before())
    this->add_producer<T>(tid, defer_spec, false);

  this->set_element<T>(tid, std::forward<ValueParam>(val));
}


//////////////////////////////////////////////////////////////////////
/// @brief Function object encapsulating promised based notification.
/// It is a  notifier which holds the promise to be fulfilled and
/// receives but ignores parameters used for rmi based data flow.
///
/// Hand written instead of lambda / bind due to symbol size blowup
/// consistently seen during compilation.
//////////////////////////////////////////////////////////////////////
template<typename T>
class promise_notifier
{
private:
  typedef typename df_stored_type<T>::type stored_value_t;
  typedef promise<stored_value_t>          promise_t;

  promise_t m_promise;

public:
  promise_notifier(promise_t pr)
    : m_promise(std::move(pr))
  { }

  void operator()(edge_container&, size_t, size_t,
                  detail::edge_version_storage<stored_value_t> const& version,
                  bool, bool)
  { m_promise.set_value(version.value()); };

  void define_type(typer& t)
  { t.member(m_promise); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Function object encapsulating @ref promise_notifier
/// initialization via the @ref directory at the producer location.
///
/// Hand written instead of lambda / bind due to symbol size blowup
/// consistently seen during compilation.
//////////////////////////////////////////////////////////////////////
template<typename T>
class promise_notifier_adder
{
private:
  typedef promise<typename df_stored_type<T>::type> promise_t;

  location_type     m_location;
  mutable promise_t m_promise;

public:
  promise_notifier_adder(location_type location, promise_t pr)
    : m_location(location), m_promise(std::move(pr))
  { }

  void operator()(p_object& d, edge_container::index_type producer_tid) const
  {
    down_cast<edge_container&>(d).add_remote_notifier<T>(
      producer_tid, m_location,
      promise_notifier<T>(std::move(m_promise)), true);
  }

  void define_type(typer& t)
  {
    t.member(m_location);
    t.member(m_promise);
  }
}; // class promise_notifier_adder


inline
void edge_container::pulse_terminator(void) const
{
  this->tg().pulse_terminator();
}


template<typename T>
void edge_container::
setup_promise_flow(index_type producer_tid,
                   promise<typename df_stored_type<T>::type> pr)
{
  // Send notifier with edge creation request via directory to the producing
  // location.
  // Unconditionally set b_is_upgrade parameter to true to override assertions
  // if multiple promises arrive from some forwarding location within pg gang.
  this->loc_directory().invoke_where(
    promise_notifier_adder<T>(this->get_location_id(), std::move(pr)),
    producer_tid);
}


template<typename T, typename Filter>
void edge_container::
receive_value(const index_type tid,
              detail::edge_entry_wrapper<T> entry,
              typename df_stored_type<
                typename boost::result_of<
                  Filter(typename df_stored_type<T>::type)
                >::type
              >::type const& val,
              const unsigned int version_id)
{
  auto entry_ptr = resolve_entry_pointer_on_flow(tid, entry.pointer());

  if (entry_ptr == nullptr)
    return;

  this->start_hold();

  entry->template set_value<Filter>(val, *this, version_id);
  this->try_eviction<T>(*entry_ptr);

  this->stop_hold();
}


template<typename T, typename Filter>
void edge_container::
receive_move_value(const index_type tid,
                   detail::edge_entry_wrapper<T> entry,
                   typename df_stored_type<
                     typename boost::result_of<
                       Filter(typename df_stored_type<T>::type)>::type
                   >::type&& val,
                   const unsigned int version_id)
{
  auto entry_ptr = resolve_entry_pointer_on_flow(tid, entry.pointer());

  if (entry_ptr == nullptr)
    return;

  this->start_hold();

  entry->template set_value<Filter>(std::move(val), *this, version_id);
  this->try_eviction<T>(*entry_ptr);

  this->stop_hold();
}


template<typename T, typename Filter>
void edge_container::
receive_shared_value(const index_type tid,
                     detail::edge_entry_wrapper<T> entry,
                     immutable_shared<
                       typename df_stored_type<T>::type> const& wrapper,
                     const unsigned int version_id)
{
  auto entry_ptr = resolve_entry_pointer_on_flow(tid, entry.pointer());

  if (entry_ptr == nullptr)
    return;

  this->start_hold();

  entry->set_shared_value(wrapper, *this, version_id);
  this->try_eviction<T>(*entry_ptr);

  this->stop_hold();
}


//////////////////////////////////////////////////////////////////////
/// @brief Function object encapsulating @ref remote_notifier
/// initialization via the @ref directory at the producer location.
///
/// Hand written instead of lambda / bind due to symbol size blowup
/// consistently seen during compilation.
//////////////////////////////////////////////////////////////////////
template<typename T, typename Filter>
class remote_notifier_adder
{
private:
  using notifier_t =
    detail::edge_notifier_wrapper<
      T, detail::edge_remote_notifier<T, Filter>>;

  location_type m_location;
  notifier_t    m_notifier;
  bool          m_b_is_upgrade;

public:
  remote_notifier_adder(location_type location,
                        notifier_t notifier,
                        bool b_is_upgrade)
    : m_location(location),
      m_notifier(std::move(notifier)),
      m_b_is_upgrade(b_is_upgrade)
  { }

  void operator()(p_object& d, edge_container::index_type producer_tid) const
  {
    down_cast<edge_container&>(d).add_remote_notifier<T>(
      producer_tid, m_location, m_notifier, m_b_is_upgrade);
  }

  void define_type(typer& t)
  {
    t.member(m_location);
    t.member(m_notifier);
    t.member(m_b_is_upgrade);
  }
}; // class remote_notifier_adder


// Construct a callback notifier object to forward to this producer's
// location via the directory.  The producer will decrement his unknonwn
// successor count and invoke this notifier (with the produced value as
// parameter).
template<typename T, typename Filter>
inline
void edge_container::send_remote_notifier(const index_type producer_tid,
                                          detail::edge_entry_wrapper<T> entry,
                                          Filter const& filter,
                                          const unsigned int version_id,
                                          const bool b_is_upgrade,
                                          const bool add_local)
{
  // notifier_t is invoked on the producer side.  It triggers an async_rmi
  // invocation to my location (e.g., receive_value()).  One of the arguments
  // to async is filter(produced_value) (executed on producer location).
  typedef detail::edge_remote_notifier<T, Filter>              notifier_t;
  typedef detail::edge_notifier_wrapper<T, notifier_t>         wrapper_t;

  // Construct a functor that will invoke add_remote_notifier (parameterized
  // with above callback notifier.  This is passed off to the directory to
  // forward to the location where producer tid is going to execute.
  if (add_local)
    this->add_remote_notifier<T>(
      producer_tid, this->get_location_id(),
      wrapper_t(notifier_t(filter, version_id, entry)), b_is_upgrade);
  else
    this->loc_directory().invoke_where(
      remote_notifier_adder<T, Filter>(
        this->get_location_id(),
        wrapper_t(notifier_t(filter, version_id, entry)),
        b_is_upgrade),
      producer_tid);
}

} // namespace stapl

#endif // ifndef STAPL_PARAGRAPH_EDGE_CONTAINER_HPP


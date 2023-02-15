/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_EXECUTOR_SCHEDULER_WORK_STEALING_SCHEDULER_HPP
#define STAPL_RUNTIME_EXECUTOR_SCHEDULER_WORK_STEALING_SCHEDULER_HPP

#include "sched_entry.hpp"
#include "steal_policies.hpp"
#include "task_placement.hpp"
#include "../../new.hpp"
#include "../../p_object.hpp"
#include <stapl/utility/down_cast.hpp>
#include <deque>
#include <iosfwd>
#include <utility>
#include <stapl/paragraph/tasks/task.h>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Work-stealing scheduling information.
///
/// Scheduling information classes that use the @ref work_stealing_sched_entry
/// have either to extend from this class or provide the @c set_migrated() and
/// @c is_migrated() functions.
///
/// @ingroup workstealing
//////////////////////////////////////////////////////////////////////
class work_stealing_sched_info
{
public:
  typedef void enable_migration;

private:
  bool m_is_migrated;

public:
  constexpr work_stealing_sched_info(none_t = none) noexcept
    : m_is_migrated(false)
  { }

  void set_migrated(void) noexcept
  { m_is_migrated = true; }

  constexpr bool is_migrated(void) const noexcept
  { return m_is_migrated; }

  void define_type(typer& t)
  { t.member(m_is_migrated); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class for entries in a work-stealing scheduler.
///
/// @tparam SchedInfo Scheduling information object type.
/// @tparam Hook      Hook for storing the entry in an intrusive container.
///
/// @see sched_entry
/// @ingroup workstealing
///
/// @todo This class requires knowledge about the task type it stores. How can
///       it be removed?
//////////////////////////////////////////////////////////////////////
template<typename SchedInfo, typename Hook>
class work_stealing_sched_entry
: public sched_entry<SchedInfo, Hook>
{
private:
  typedef paragraph_impl::task_base_intermediate<
    work_stealing_sched_entry
  > derived_task_type;

public:
  explicit work_stealing_sched_entry(SchedInfo const& info)
  : sched_entry<SchedInfo, Hook>(info)
  { }

  explicit work_stealing_sched_entry(SchedInfo&& info) noexcept
  : sched_entry<SchedInfo, Hook>(std::move(info))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the underlying task can be migrated.
  //////////////////////////////////////////////////////////////////////
  bool is_migratable(void) const noexcept
  {
    return static_cast<derived_task_type const*>(this)->migratable();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Migrates the task to the given location.
  //////////////////////////////////////////////////////////////////////
  void migrate(const location_type dest, task_graph* tg_ptr)
  {
    this->sched_info().set_migrated();
    static_cast<derived_task_type*>(this)->migrate(dest, tg_ptr);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Scheduler that provides work stealing support.
///
/// The default policy is circular stealing through @ref circular_steal.
///
/// @ingroup workstealing
///
/// @todo It is not clear if the scheduler should be a @ref p_object or the
///       executor.
/// @todo The polling interval never decreases.
//////////////////////////////////////////////////////////////////////
template<typename StealPolicy = circular_steal>
class work_stealing_scheduler
  : public default_task_placement,
    public p_object
{
private:
  typedef StealPolicy                        stealer_type;
  typedef boost::intrusive::list_base_hook<> hook_type;
public:
  typedef work_stealing_sched_info           sched_info_type;
  typedef work_stealing_sched_entry<
            sched_info_type, hook_type
          >                                  entry_type;
  typedef typename boost::intrusive::make_list<
            entry_type,
            boost::intrusive::cache_last<true>,
            boost::intrusive::constant_time_size<true>
          >::type                            list_type;
  typedef std::deque<location_type>          thieves_list_type;
  typedef std::size_t                        size_type;

private:
  list_type         m_mig_entries;
  list_type         m_nonmig_entries;
  size_type         m_executed;

  stealer_type      m_stealer;
  thieves_list_type m_thieves;

  size_type         m_poll_interval;
  const size_type   m_poll_min;
  const size_type   m_poll_max;
  const size_type   m_poll_step;

  size_type         m_poll_count;
  size_type         m_req_sent;
  size_type         m_req_received;
  size_type         m_failed_steals;
  size_type         m_loot_received;
  size_type         m_notifications_received;

  task_graph*       m_tg_ptr;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Creates a new @ref work_stealing_scheduler.
  ///
  /// @param chunk_size Number of entries to steal with one request.
  /// @param fraction   Fraction of the total entries to steal.
  /// @param poll_min   Minimum number of calls to process stolen work before
  ///                   yielding.
  /// @param poll_max   Maximum number of calls to process stolen work before
  ///                   yielding.
  /// @param poll_step  How many times to increase the number of calls to
  ///                   process stolen work before yielding.
  //////////////////////////////////////////////////////////////////////
  work_stealing_scheduler(size_type chunk_size,
                          size_type fraction,
                          size_type poll_min,
                          size_type poll_max,
                          size_type poll_step)
   : p_object(no_aggregation | no_fence_information | allow_try_rmi),
     m_executed(0),
     m_stealer(this->get_location_id(),
               this->get_num_locations(),
               chunk_size,
               fraction),
     m_poll_interval(poll_min),
     m_poll_min(poll_min),
     m_poll_max(poll_max),
     m_poll_step(poll_step),
     m_poll_count(0),
     m_req_sent(0),
     m_req_received(0),
     m_failed_steals(0),
     m_loot_received(0),
     m_notifications_received(0),
     m_tg_ptr(nullptr)
  {
    stapl_assert((m_poll_min <= m_poll_max && m_poll_min > 0),
                 "Invalid polling parameters.");
  }

  work_stealing_scheduler(work_stealing_scheduler const& other)
    : p_object(no_aggregation | no_fence_information | allow_try_rmi),
      m_executed(0),
      m_stealer(other.m_stealer),
      m_poll_interval(other.m_poll_interval),
      m_poll_min(other.m_poll_min),
      m_poll_max(other.m_poll_max),
      m_poll_step(other.m_poll_step),
      m_poll_count(other.m_poll_count),
      m_req_sent(other.m_req_sent),
      m_req_received(other.m_req_received),
      m_failed_steals(other.m_failed_steals),
      m_loot_received(other.m_loot_received),
      m_notifications_received(other.m_notifications_received),
      m_tg_ptr(other.m_tg_ptr)
  {
    stapl_assert(other.empty_impl(), "Scheduler has entries");
    stapl_assert(other.m_executed == 0, "other.m_executed non zero");
    stapl_assert(other.m_thieves.empty(), "other.m_thieves is not empty");
    stapl_assert(other.m_poll_count == 0, "other.m_poll_count in non zero");
    stapl_assert(   other.m_req_sent == 0
                 && other.m_req_received == 0
                 && other.m_failed_steals == 0
                 && other.m_loot_received == 0
                 && other.m_notifications_received == 0,
                 "why is there any diagnostic information yet?"
                );
  }

  void set_tg(task_graph& tg) noexcept
  {
    m_tg_ptr = &tg;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an active entry to the scheduler.
  ///
  /// If the entry is migrated from somewhere else, it also notifies about that
  /// fact.
  //////////////////////////////////////////////////////////////////////
  void add_active(entry_type& e)
  {
    if (e.is_migratable())
      m_mig_entries.push_front(e);
    else
      m_nonmig_entries.push_front(e);

    // inform the stealer if this is a migrated entry
    if (e.sched_info().is_migrated()) {
      m_stealer.receive_work_notification();
    }
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an idle entry to the scheduler.
  //////////////////////////////////////////////////////////////////////
  void add_idle(entry_type&) noexcept
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the next entry to be executed.
  //////////////////////////////////////////////////////////////////////
  entry_type& next(void)
  {
    // process non migratable entries
    if (!m_nonmig_entries.empty()) {
      entry_type& e = m_nonmig_entries.front();
      m_nonmig_entries.pop_front();
      ++m_executed;

      // try and process some steal requests
      try_process_steal_requests();
      return e;
    }

    // process migratable entries
    stapl_assert(!m_mig_entries.empty(), "scheduler is empty");
    entry_type& e = m_mig_entries.front();
    m_mig_entries.pop_front();
    ++m_executed;

    // try and process some steal requests
    try_process_steal_requests();
    return e;
  }

private:
  bool empty_impl(void) const noexcept
  { return (m_nonmig_entries.empty() && m_mig_entries.empty()); }

public:
  bool empty(void)
  {
    // local work is done. Post some steal requests
    if (empty_impl()) {
      m_stealer.steal(*this);
      // free locations who have posted steal requests
      try_process_steal_requests();
    }
    // inform the executor that the local work queue is empty
    return empty_impl();
  }

  bool ready(void)
  {
    return !empty();
  }

  size_type size(void) const noexcept
  { return (m_nonmig_entries.size() + m_mig_entries.size()); }

  list_type const& migratable_entries(void) const noexcept
  { return m_mig_entries; }

  list_type& migratable_entries(void) noexcept
  { return m_mig_entries; }

  thieves_list_type& thieves(void) noexcept
  { return m_thieves; }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Process steal requests if there are any available.
  //////////////////////////////////////////////////////////////////////
  void try_process_steal_requests(void)
  {
    const bool avoided_yield = runtime::yield_if_not(
                                 [&]
                                 { return ((m_executed%m_poll_interval)!=0); });
    if (!avoided_yield)
      ++m_poll_count;

    if (m_thieves.empty()) {
      // no steal request, decrease polling (increase polling interval)
      if (m_poll_interval!=m_poll_max)
        m_poll_interval = std::min(m_poll_max, (m_poll_interval * m_poll_step));
    }
    else {
      // invoke the stealer to process the steal requests
      m_stealer.process_steal_requests(*this, m_tg_ptr);
    }
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Sends a steal request to the given destination.
  //////////////////////////////////////////////////////////////////////
  void request_steal(size_type dest)
  {
    ++m_req_sent;

    gang g{*this}; // count traffic in PARAGRAPH's gang

    // termination detection may succeed while we are trying to steal
    // hence the use of try_rmi for steal requests
    try_rmi(dest, this->get_rmi_handle(),
            &work_stealing_scheduler::steal_request, this->get_location_id());
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Called remotely to process a steal request.
  ///
  /// @see work_stealing_scheduler::request_steal
  //////////////////////////////////////////////////////////////////////
  void steal_request(location_type loc)
  {
    ++m_req_received;

    // if we have any work add this location to the list of thieves
    if (m_stealer.has_work(*this)) {
      m_thieves.push_back(loc);
    }
    else {
      // no need to keep this location waiting
      notify_steal_completion(loc, 0);
    }
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Notifies of successful steal request completion to the destination.
  //////////////////////////////////////////////////////////////////////
  void notify_steal_completion(location_type dest, size_type loot)
  {
    gang g{*this}; // count traffic in PARAGRAPH's gang

    // termination detection may succeed while we are trying to steal
    // hence the use of try_rmi for steal requests
    try_rmi(dest, this->get_rmi_handle(),
            &work_stealing_scheduler::receive_steal_completion,
            this->get_location_id(), loot);
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Called remotely to process a notification for a successful steal
  ///        request.
  ///
  /// @see work_stealing_scheduler::notify_steal_completion()
  //////////////////////////////////////////////////////////////////////
  void receive_steal_completion(location_type loc, size_type loot) noexcept
  {
    ++m_notifications_received;
    m_loot_received += loot;

    if (loot == 0)
      ++m_failed_steals;

    m_stealer.receive_steal_completion(loc, loot);
  }

public:
  friend std::ostream& operator<<(std::ostream& os,
                                  work_stealing_scheduler const& s)
  {
    return os << "PID " << s.get_location_id() << ": ws-scheduler:"
              << " m_executed = "               << s.m_executed
              << " m_poll_interval = "          << s.m_poll_interval
              << " m_poll_min = "               << s.m_poll_min
              << " m_poll_max = "               << s.m_poll_max
              << " m_poll_step = "              << s.m_poll_step
              << " m_poll_count = "             << s.m_poll_count
              << " m_req_sent = "               << s.m_req_sent
              << " m_req_received = "           << s.m_req_received
              << " m_failed_steals = "          << s.m_failed_steals
              << " m_loot_received = "          << s.m_loot_received
              << " m_notifications_received = " << s.m_notifications_received
              << s.m_stealer;
  }
};

} // namespace stapl

#endif

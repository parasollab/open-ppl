/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_EXECUTOR_GANG_EXECUTOR_HPP
#define STAPL_RUNTIME_EXECUTOR_GANG_EXECUTOR_HPP

#include "executor_base.hpp"
#include "scheduler/default_info.hpp"
#include "../exception.hpp"
#include "../gang.hpp"
#include "../new.hpp"
#include "../system.hpp"
#include "../yield.hpp"
#include <ios>
#include <iosfwd>
#include <mutex>
#include <typeinfo>
#include <utility>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Gang associated executor.
///
/// Schedules @ref executor objects from PARAGRAPHs and @ref gang_executor
/// objects from child parallel sections.
///
/// @warning @p Scheduler::entry_type has to provide a function @c is_linked().
///
/// @ingroup executors
///
/// @todo It does not support scheduling tasks. The best way to support them is
///       to unify this class with @ref executor. This support probably also
///       requires termination detection.
///
/// @todo Replace default_info with Scheduler::sched_info_type to enable
///       scheduling on non-integral types.  This requires making the type of
///       the scheduler available outside of the gang_executor so code like
///       the down_cast in executor_base::add_executor is supported.
//////////////////////////////////////////////////////////////////////
template<typename Scheduler>
class gang_executor
: public runtime::executor_intermediate<default_info>
{
public:
  using scheduler_type   = Scheduler;
  using sched_info_type  = default_info;
  using result_type      = executor_base::result_type;
private:
  using size_type        = typename scheduler_type::size_type;
  using sched_entry_type = typename scheduler_type::entry_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Executor entry type.
  //////////////////////////////////////////////////////////////////////
  class entry_type
  : public sched_entry_type
  {
  private:
    // pointer to the executor associated with this entry
    executor_base* m_ex;
    // true if the entry is responsible for deleting the executor
    const bool     m_cleanup;

  public:
    //////////////////////////////////////////////////////////////////////
    /// @brief Creates a new entry.
    ///
    /// @param ex         Associated executor.
    /// @param sched_info Scheduling information.
    /// @param cleanup    @c true if the entry is responsible for deleting the
    ///                   executor.
    //////////////////////////////////////////////////////////////////////
    template<typename SchedInfo>
    entry_type(executor_base& ex, SchedInfo&& sched_info, const bool cleanup)
    : sched_entry_type(std::forward<SchedInfo>(sched_info)),
      m_ex(&ex),
      m_cleanup(cleanup)
    { }

  private:
    ~entry_type(void)
    {
      if (m_cleanup)
        delete m_ex;
    }

  public:
    STAPL_USE_MANAGED_ALLOC(entry_type)

    //////////////////////////////////////////////////////////////////////
    /// @brief Makes this entry stale.
    ///
    /// Upon calling this function, the chunker entry associated with the entry
    /// will be retired and the entry will be deleted if it is idle. If it is
    /// not idle, it will be deleted after it has finished executing.
    ///
    /// If the entry is not responsible for deleting the executor, the latter
    /// will be disassociated from it and the entry will be deleted at the next
    /// invocation of the function operator of the executor it is added to.
    //////////////////////////////////////////////////////////////////////
    void make_stale(void) noexcept
    {
      m_ex->retire_chunker_entry();
      if (idle())
        delete this;
      else if (!m_cleanup)
        m_ex = nullptr;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief Returns @c true if this entry is not inside a scheduler or the
    ///        function operator is not active.
    //////////////////////////////////////////////////////////////////////
    bool idle(void) const noexcept
    { return (!this->is_linked() && m_ex->idle()); }

    //////////////////////////////////////////////////////////////////////
    /// @brief Calls the underlying executor's @ref executor_base::populate()
    ///        function.
    //////////////////////////////////////////////////////////////////////
    result_type populate(void)
    {
      const auto r = m_ex->populate();
      if (r==runnable_base::Finished)
        delete this;
      return r;
    }

    //////////////////////////////////////////////////////////////////////
    /// @brief Calls the function operator of the underlying executor and
    ///        returns its result.
    ///
    /// If @ref make_stale() was called, then the entry is deleted.
    ///
    /// If this entry is responsible for cleaning-up the underlying executor,
    /// then it deletes it if the result is @ref runnable_base::Finished.
    //////////////////////////////////////////////////////////////////////
    result_type operator()(void)
    {
      if (!m_ex) {
        // stale entry, we can delete it
        delete this;
        return runnable_base::Finished;
      }
      // valid entry - call function operator
      const auto r = (*m_ex)();
      if (r==runnable_base::Finished)
        delete this;
      return r;
    }
  };

  runtime::location_md& m_location;
  scheduler_type        m_scheduler;
  chunker               m_chunker;
  // number of scheduled executor objects
  size_type             m_num_ex;

public:
  explicit gang_executor(runtime::location_md& l)
  : m_location(l),
    m_chunker(get_default_executor_window_size(),
              get_default_executor_retire_chunk()),
    m_num_ex(0)
  { }

  gang_executor(runtime::location_md& l, Scheduler&& scheduler)
  : m_location(l),
    m_scheduler(std::move(scheduler)),
    m_chunker(get_default_executor_window_size(),
              get_default_executor_retire_chunk()),
    m_num_ex(0)
  { }

  ~gang_executor(void) override
  {
    STAPL_RUNTIME_ASSERT(empty());
    this->notify_finished();
  }

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Executes pending entries.
  ///
  /// This function is called by @ref gang_executor::operator() when some
  /// (@p drain is @c false) or all (@p drain is @c true) entries have to be
  /// scheduled.
  ///
  /// It is also called by @ref gang_executor::add_executor() if the maximum
  /// number of added entries has been reached (@c STAPL_EXECUTOR_WINDOW_SIZE).
  ///
  /// @warning The function may return even if all the entries have not been
  ///          scheduled, as the processing loop is bound by the number of
  ///          pending entries. This is to ensure that every call is
  ///          non-blocking.
  //////////////////////////////////////////////////////////////////////
  result_type do_progress(const bool drain)
  {
    STAPL_RUNTIME_ASSERT(m_scheduler.ready());

    auto pending = m_chunker.pending();

    while (true) {
      size_type       iteration_cnt = 0;
      const size_type num_entries   = m_scheduler.size();

      // execute pending entries; keep track of the number of entries to avoid
      // cases where entries are being re-added to ensure non-blocking behavior
      do {
        auto& entry = static_cast<entry_type&>(m_scheduler.next());

        const auto status = entry();
        switch (status) {
          case runnable_base::Active:
            // entry has still things to do
            m_scheduler.add_active(entry);
            break;
          case runnable_base::Idle:
            // entry has something to do, but it's not runnable yet
            m_scheduler.add_idle(entry);
            break;
          case runnable_base::Finished:
            // the entry will clean-up after itself
            break;
          default:
            STAPL_RUNTIME_ERROR("Incorrect entry status.");
            break;
        }
      } while (m_scheduler.ready() && ++iteration_cnt<num_entries);

      m_chunker.cleanup();

      if (!drain)
        break;

      if (!m_scheduler.ready())
        return runnable_base::Idle;

      // exit loop if number of pending entries did not change
      const auto new_pending = m_chunker.pending();
      if (pending==new_pending)
        break;
      pending = new_pending;
    }

    return (!m_scheduler.ready() ? runnable_base::Idle : runnable_base::Active);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Marks @p entry runnable.
  //////////////////////////////////////////////////////////////////////
  void add_active_entry(entry_type& entry)
  {
    m_scheduler.add_active(entry);
    this->notify_runnable();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_executor(executor_base*,SchedInfo&&,const bool)
  ///
  /// @bug It causes the sliding window to increase past the bounds if the added
  ///      executor is a @ref gang_executor or if this executor is already
  ///      active.
  //////////////////////////////////////////////////////////////////////
  template<typename SchedInfo>
  void add(executor_base* ex, SchedInfo&& sched_info, bool is_gang_executor)
  {
    // avoid inserting an executor to itself
    STAPL_RUNTIME_ASSERT(static_cast<executor_base*>(this)!=ex);

    entry_type* const entry = new entry_type{*ex,
                                       std::forward<SchedInfo>(sched_info),
                                       !is_gang_executor};

    if (is_gang_executor) {
      if (ex->empty())
        STAPL_RUNTIME_ERROR("Added gang_executor is empty.");

      // create notifiers for re-addition and deletion
      ex->set_notifiers([this, entry]
                        {
                          if (entry->idle())
                            this->add_active_entry(*entry);
                        },
                        m_chunker.get_entry(),
                        [entry]
                        {
                          entry->make_stale();
                        });

      // add entry to the scheduler
      add_active_entry(*entry);
    }
    else {
      // If we've maxed out our window size and we are not executing, retire
      // some entries before moving forward with initializing this one.
      if (this->idle() && m_chunker.full() && m_scheduler.ready()) {
        gang g{*this};
        std::lock_guard<decltype(this->m_executing)> eg{this->m_executing};
        do_progress(false);
      }

      // separate count of executors to enforce drain
      ++m_num_ex;

      // create notifier only for re-addition
      ex->set_notifiers([this, entry]
                        {
                          if (entry->idle())
                            this->add_active_entry(*entry);
                        },
                        m_chunker.get_entry(),
                        [this, entry]
                        {
                          --(this->m_num_ex);
                          entry->make_stale();
                        });

      // execute the factory task
      result_type status;
      {
        gang g{*this};
        std::lock_guard<decltype(this->m_executing)> eg{this->m_executing};
        status = entry->populate();
      }

      switch (status) {
        case runnable_base::Active:
          // entry has still things to do
          add_active_entry(*entry);
          break;
        case runnable_base::Idle:
          // entry is not runnable but future events (e.g. executor::add_task())
          // will change state
          m_scheduler.add_idle(*entry);
          break;
        case runnable_base::Finished:
          // factory generated no tasks and termination detection succeeded
          // (probably due to specialized local only termination detection)
          break;
        default:
          STAPL_RUNTIME_ERROR("Incorrect entry status.");
          break;
      }

      m_chunker.cleanup();
    }
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_task(runnable_base*,none_t)
  //////////////////////////////////////////////////////////////////////
  void add_task_impl(runnable_base*) override
  { STAPL_RUNTIME_ERROR("Not implemented."); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_executor(executor_base*,SchedInfo&&,const bool)
  //////////////////////////////////////////////////////////////////////
  void add_executor_impl(executor_base* ex,
                         sched_info_type const& sched_info,
                         const bool is_gang_executor) override
  { add(ex, sched_info, is_gang_executor); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_executor(executor_base*,SchedInfo&&,const bool)
  //////////////////////////////////////////////////////////////////////
  void add_executor_impl(executor_base* ex,
                         sched_info_type&& sched_info,
                         const bool is_gang_executor) override
  { add(ex, std::move(sched_info), is_gang_executor); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_executor(executor_base*,none_t,const bool)
  //////////////////////////////////////////////////////////////////////
  void add_executor_impl(executor_base* ex,
                         const bool is_gang_executor) override
  { add(ex, sched_info_type{}, is_gang_executor); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::print(std::ostream&)
  //////////////////////////////////////////////////////////////////////
  void print(std::ostream& os) override
  {
    os << std::boolalpha
       << runtime::demangle(typeid(*this).name())              << ':'
       << " addr("               << this                       << ')'
       << " belongs_to("
         << m_location.get_gang_md().get_id() << ',' << m_location.get_id()
       << ')'
       << " window_size("        << m_chunker.max_size()       << ')'
       << " retire_chunk("       << m_chunker.max_chunk_size() << ')'
       << " is_empty("           << empty()                    << ')'
       << " is_scheduler_empty(" << m_scheduler.empty()        << ')'
       << " pending_entries("    << m_chunker.pending()        << ')'
       << std::noboolalpha;
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief @ref gang_executor objects do not get populated by any of the
  ///        entries. This function is here to enforce interface compatibility.
  //////////////////////////////////////////////////////////////////////
  result_type populate(void) noexcept override
  {
    STAPL_RUNTIME_ERROR("Invalid call for gang_executor.");
    return runnable_base::Idle;
  }

  result_type operator()(void) override
  {
    if (!m_scheduler.ready())
      return runnable_base::Idle;

    gang g{*this};
    std::lock_guard<decltype(this->m_executing)> eg{this->m_executing};
    return do_progress(false);
  }

  void operator()(execute_all_t) override
  {
    gang g{*this};
    std::lock_guard<decltype(this->m_executing)> eg{this->m_executing};

    while (m_num_ex>0) {
      // block until there entries in the scheduler appear or there are no more
      // PARAGRAPH executors
      const bool yielded = runtime::yield_until(
        [this]
        {
          return (this->m_scheduler.ready() || this->m_num_ex==0);
        });

      // if entries appeared, process them
      bool active = false;
      if (m_scheduler.ready())
        active = (do_progress(true)==runnable_base::Active);

      // if previous call did not yield and the executor is still active, do a
      // proactive yield as it is probably waiting for an RMI to arrive
      runtime::yield_if_not([yielded, active] { return (yielded || !active); });
    }
  }

  bool empty(void) noexcept override
  {
    m_chunker.cleanup();
    return m_chunker.empty();
  }

  runtime::location_md const& get_location_md(void) const noexcept
  { return m_location; }

  runtime::location_md& get_location_md(void) noexcept
  { return m_location; }

  scheduler_type const& get_scheduler(void) const noexcept
  { return m_scheduler; }

  scheduler_type& get_scheduler(void) noexcept
  { return m_scheduler; }
};

} // namespace stapl

#endif

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_EXECUTOR_EXECUTOR_HPP
#define STAPL_RUNTIME_EXECUTOR_EXECUTOR_HPP

#ifdef STAPL_CALLGRIND_PROFILE_TASK_EXECUTION
#  include <valgrind/callgrind.h>
#endif

#include "executor_base.hpp"
#include "terminator_base.hpp"
#include "../gang.hpp"
#include "../new.hpp"
#include "../system.hpp"
#include "../yield.hpp"
#include <ios>
#include <iosfwd>
#include <mutex>
#include <typeinfo>
#include <utility>

#include <stapl/utility/down_cast.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Executor for scheduling runnable tasks with specific scheduling
///        information.
///
/// @tparam TaskGraph Task graph type.
///
/// @ingroup executors
///
/// @todo Does not support scheduling executors. Needs to be unified with
///       @ref gang_executor.
//////////////////////////////////////////////////////////////////////
template<typename TaskGraph>
class executor
  : public runtime::executor_intermediate<
             typename TaskGraph::scheduler_type::sched_info_type
           >
{
public:
  using scheduler_type  = typename TaskGraph::scheduler_type;
  using sched_info_type = typename scheduler_type::sched_info_type;
  using result_type     = executor_base::result_type;
private:
  using task_type       = typename TaskGraph::task_type;

  scheduler_type   m_scheduler;
  TaskGraph&       m_tg;
  bool             m_finished;

public:
  template<typename Scheduler>
  executor(TaskGraph& tg, Scheduler&& s)
    : m_scheduler(std::forward<Scheduler>(s)),
      m_tg(tg),
      m_finished(false)
  { }

  ~executor(void) override
  {
    STAPL_RUNTIME_ASSERT(m_finished && empty());
  }

  STAPL_USE_MANAGED_ALLOC(executor)

private:
  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the status of this @ref executor.
  ///
  /// @return @ref runnable_base::Active if there are more entries,
  ///         @ref runnable_base::Idle if there are might be entries in the
  ///         future or
  ///         @ref runnable_base::Finished if there is nothing else to be done.
  //////////////////////////////////////////////////////////////////////
  result_type status(void) noexcept
  {
    if (m_finished) {
      STAPL_RUNTIME_ASSERT(m_scheduler.empty());
      return runnable_base::Finished;
    }

    // if empty, waiting on more entries or termination detection
    return (!m_scheduler.ready() ? runnable_base::Idle : runnable_base::Active);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Executes pending entries.
  ///
  /// Flushes the queue of runnable entries and queries the task graph. If more
  /// entries are added, it runs another iteration, flushing the run queue. It
  /// repeats the same process until there are no more runnable entries.
  ///
  /// @return @ref runnable_base::Idle if there are might be entries in the
  ///         future or
  ///         @ref runnable_base::Finished if termination detection succeeded.
  ///
  /// @todo @c TaskGraph::query_factory() forces the executor to remain active,
  ///       instead of properly returning whatever its status is. This is done
  ///       to support cases such as the do-while skeleton that use the
  ///       @c TaskGraph::query_factory() as a callback for the creation of
  ///       tasks later in the computation.
  //////////////////////////////////////////////////////////////////////
  result_type do_progress(void)
  {
    STAPL_RUNTIME_ASSERT(!m_finished);

    // Flush the queue of runnable entries and then query factory. If it added
    // runnable entries or the factory needs to be reinvoked, run another
    // iteration of outer loop, flushing the run queue.
    // Repeat until there are no more runnable entries or the factory does not
    // need to be re-queried.

    auto& ctx = runtime::this_context::get();

    do {
      p_object const* prev_comm_p_object = nullptr;

      boost::optional<gang> task_gang;

      while (m_scheduler.ready()) {

#ifdef STAPL_CALLGRIND_PROFILE_TASK_EXECUTION
        CALLGRIND_START_INSTRUMENTATION;
#endif

        task_type& task = static_cast<task_type&>(m_scheduler.next());

        p_object const& comm_p_object = task.comm_p_object(&m_tg);

        if (prev_comm_p_object != &comm_p_object)
        {
          task_gang          = boost::in_place<gang>(comm_p_object);
          prev_comm_p_object = &comm_p_object;
        }

        task(&m_tg);
        m_scheduler.notify_finished();

        runtime::scheduling_point(ctx);

#ifdef STAPL_CALLGRIND_PROFILE_TASK_EXECUTION
        CALLGRIND_STOP_INSTRUMENTATION;
        CALLGRIND_DUMP_STATS;
#endif

      }

      task_gang = boost::none;

      // if termination detection succeeded since the last task finished
      // and the runtime scheduling point, return finished.
      if (this->m_finished)
      {
        STAPL_RUNTIME_ASSERT(m_scheduler.empty());
        return runnable_base::Finished;
      }

      // if the callback claims that there are more tasks to be added, but
      // none were, keep the executor active (see @todo)
      if (m_tg.query_factory() && !m_scheduler.ready())
        return runnable_base::Active;

    } while (m_scheduler.ready());

    return status();
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_task(runnable_base*,none_t)
  //////////////////////////////////////////////////////////////////////
  void add_task_impl(runnable_base* t) override
  {
    task_type* task_ptr = down_cast<task_type*>(t);
    m_scheduler.add_active(*task_ptr);
    this->notify_runnable();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_executor(executor_base*,SchedInfo&&,const bool)
  //////////////////////////////////////////////////////////////////////
  void add_executor_impl(executor_base*,
                         sched_info_type const&,
                         const bool is_gang_executor) override
  { STAPL_RUNTIME_ERROR("Not implemented."); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_executor(executor_base*,SchedInfo&&,const bool)
  //////////////////////////////////////////////////////////////////////
  void add_executor_impl(executor_base*,
                         sched_info_type&&,
                         const bool is_gang_executor) override
  { STAPL_RUNTIME_ERROR("Not implemented."); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_executor(executor_base*,none_t,const bool)
  //////////////////////////////////////////////////////////////////////
  void add_executor_impl(executor_base*,
                         const bool is_gang_executor) override
  { STAPL_RUNTIME_ERROR("Not implemented."); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::print(std::ostream&)
  //////////////////////////////////////////////////////////////////////
  void print(std::ostream& os) override
  {
    const char* st = "Unknown";
    switch (status()) {
      case runnable_base::Active:
        st = "Active";
        break;
      case runnable_base::Idle:
        st = "Idle";
        break;
      case runnable_base::Finished:
        st = "Finished";
        break;
      default:
        st = "INCORRECT";
        break;
    }

    os << std::boolalpha
       << runtime::demangle(typeid(*this).name()) << ':'
       << " addr("        << this                 << ')'
       << " task_graph("  << &m_tg                << ')'
       << " status("      << st                   << ')'
       << " is_empty("    << m_scheduler.empty()  << ')'
       << " is_finished(" << m_finished           << ')'
       << std::noboolalpha;
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Calls the notifier that informs that this @ref executor has
  ///        finished.
  //////////////////////////////////////////////////////////////////////
  void notify_finished(void)
  {
    STAPL_RUNTIME_ASSERT(!m_finished && m_scheduler.empty());
    m_finished = true;
    executor_base::notify_finished();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Populates the executor with entries if there is a factory task.
  ///
  /// @todo The @ref paragraph should provide some hint if there is a factory
  ///       task to avoid unnecessary calls.
  //////////////////////////////////////////////////////////////////////
  result_type populate(void) override
  {
    std::lock_guard<decltype(this->m_executing)> eg{this->m_executing};

    STAPL_RUNTIME_ASSERT(!m_finished);

    if (m_scheduler.ready()) {
      task_type& task = static_cast<task_type&>(m_scheduler.next());

      if (!task.is_factory_task()) {
        // Only factory tasks should execute outside purview of the parent's
        // scheduler. Put this task back in the scheduler, and return active to
        // the parent since there is at least one runnable task.
        m_scheduler.add_active(task);
        return runnable_base::Active;
      }

      // populate with a factory call
      STAPL_RUNTIME_ASSERT_MSG(m_scheduler.empty(),
                               "Found factory_task with other tasks.");
      task(&m_tg);
    }

    const auto r = status();

    if (r==runnable_base::Idle)
      return m_tg.query_factory() ? runnable_base::Active : status();

    return r;
  }

  result_type operator()(void) override
  {
    std::lock_guard<decltype(this->m_executing)> eg{this->m_executing};
    return do_progress();
  }

  void operator()(execute_all_t) override
  {
    std::lock_guard<decltype(this->m_executing)> eg{this->m_executing};
    runtime::yield_until(
      [this]
      {
        if (this->m_finished)
          return true;
        return (this->do_progress()==runnable_base::Finished);
      });
  }

  bool empty(void) override
  { return m_scheduler.empty(); }

  scheduler_type const& scheduler(void) const noexcept
  { return m_scheduler; }

  scheduler_type& scheduler(void) noexcept
  { return m_scheduler; }
};

} // namespace stapl

#endif

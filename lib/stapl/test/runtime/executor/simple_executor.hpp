/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_RUNTIME_TEST_SIMPLE_EXECUTOR_HPP
#define STAPL_RUNTIME_TEST_SIMPLE_EXECUTOR_HPP

#include <stapl/runtime/executor/executor_base.hpp>
#include <stapl/runtime/system.hpp>
#include <functional>
#include <typeinfo>
#include <utility>

//////////////////////////////////////////////////////////////////////
/// @brief Simple executor based on @ref stapl::executor.
///
/// @todo Replace with @ref stapl::executor when it does not depend on the
///       @ref stapl::task_graph anymore.
//////////////////////////////////////////////////////////////////////
template<typename Scheduler>
class simple_executor
: public stapl::runtime::executor_intermediate<
           typename Scheduler::sched_info_type
         >
{
public:
  using scheduler_type  = Scheduler;
  using sched_info_type = typename scheduler_type::sched_info_type;
  using result_type     = stapl::executor_base::result_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Simple task type for @ref simple_executor.
  //////////////////////////////////////////////////////////////////////
  class task_type
  : public stapl::runnable_base
  {
  private:
    std::function<void(void)> m_function;

  public:
    template<typename Function>
    explicit task_type(Function&& f)
    : m_function(std::forward<Function>(f))
    { }

    result_type operator()(void) const
    {
      m_function();
      return stapl::runnable_base::Finished;
    }
  };

private:
  using sched_entry_type = typename scheduler_type::entry_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Entry class for @ref simple_executor.
  //////////////////////////////////////////////////////////////////////
  class entry_type
  : public sched_entry_type
  {
  private:
    stapl::runnable_base* m_p;

  public:
    //////////////////////////////////////////////////////////////////////
    /// @brief Creates a new entry from a task.
    ///
    /// @param p    Pointer to the task.
    /// @param info Scheduling information.
    //////////////////////////////////////////////////////////////////////
    entry_type(stapl::runnable_base* p, sched_info_type const& info)
      : sched_entry_type(info), m_p(p)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @copydoc entry_type(stapl::runnable_base*,sched_info_type const&)
    //////////////////////////////////////////////////////////////////////
    entry_type(stapl::runnable_base* p, sched_info_type&& info) noexcept
      : sched_entry_type(std::move(info)), m_p(p)
    { }

    //////////////////////////////////////////////////////////////////////
    /// @brief Calls the function operator of the underlying task and returns
    ///        its result.
    ///
    /// The task is not deleted as it is the responsibility of the
    /// @ref paragraph.
    //////////////////////////////////////////////////////////////////////
    result_type operator()(void)
    {
      const auto r = static_cast<task_type&>(*m_p)();
      STAPL_RUNTIME_ASSERT(r==stapl::runnable_base::Finished);
      delete this;
      return r;
    }
  };

private:
  Scheduler m_scheduler;

public:
  ~simple_executor(void)
  {
    if (!empty())
      STAPL_RUNTIME_ERROR("simple_executor not empty.");
  }

protected:
  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_task(runnable_base*,SchedInfo const&)
  //////////////////////////////////////////////////////////////////////
  void add_task_impl(stapl::runnable_base* t, sched_info_type const& sched_info)
  {
    auto* const entry = new entry_type(t, sched_info);
    m_scheduler.add_active(*entry);
    this->notify_runnable();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_task(runnable_base*,SchedInfo const&)
  //////////////////////////////////////////////////////////////////////
  void add_task_impl(stapl::runnable_base* t, sched_info_type&& sched_info)
  {
    auto* const entry = new entry_type(t, std::move(sched_info));
    m_scheduler.add_active(*entry);
    this->notify_runnable();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_task(runnable_base*,none_t)
  //////////////////////////////////////////////////////////////////////
  void add_task_impl(stapl::runnable_base* t)
  {
    auto* const entry = new entry_type(t, stapl::none);
    m_scheduler.add_active(*entry);
    this->notify_runnable();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_executor(executor_base*,SchedInfo const&,const bool)
  //////////////////////////////////////////////////////////////////////
  void add_executor_impl(stapl::executor_base*,
                         sched_info_type const&,
                         const bool is_gang_executor)
  { STAPL_RUNTIME_ERROR("Not implemented."); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_executor(executor_base*,SchedInfo const&,const bool)
  //////////////////////////////////////////////////////////////////////
  void add_executor_impl(stapl::executor_base*,
                         sched_info_type&&,
                         const bool is_gang_executor)
  { STAPL_RUNTIME_ERROR("Not implemented."); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::add_executor(executor_base*,none_t,const bool)
  //////////////////////////////////////////////////////////////////////
  void add_executor_impl(stapl::executor_base*,
                         const bool is_gang_executor)
  { STAPL_RUNTIME_ERROR("Not implemented."); }

  result_type status(void)
  {
    if (empty())
      return stapl::runnable_base::Finished;
    return stapl::runnable_base::Active;
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc executor_base::print(std::ostream&)
  //////////////////////////////////////////////////////////////////////
  void print(std::ostream& os)
  {
    const char* st = "Unknown";
    switch (status()) {
      case stapl::runnable_base::Active:
        st = "Active";
        break;
      case stapl::runnable_base::Idle:
        st = "Idle";
        break;
      case stapl::runnable_base::Finished:
        st = "Finished";
        break;
      default:
        st = "INCORRECT";
        break;
    }

    os << std::boolalpha
       << stapl::runtime::demangle(typeid(*this).name()) << ':'
       << " addr("     << this    << ')'
       << " is_empty(" << empty() << ')'
       << " status("   << st      << ')'
       << std::noboolalpha;
  }

  result_type do_progress(void)
  {
    if (empty())
      STAPL_RUNTIME_ERROR("simple_executor is empty.");
    auto& entry = static_cast<entry_type&>(m_scheduler.next());
    entry();
    return status();
  }

public:
  result_type populate(void)
  {
    return (empty() ? stapl::runnable_base::Idle
                    : stapl::runnable_base::Active);
  }

  result_type operator()(void)
  {
    std::lock_guard<decltype(this->m_executing)> eg{this->m_executing};
    return do_progress();
  }

  void operator()(stapl::execute_all_t)
  {
    std::lock_guard<decltype(this->m_executing)> eg{this->m_executing};
    stapl::block_until(
      [this] { return (this->do_progress()==stapl::runnable_base::Finished); });
  }

  bool empty(void)
  { return m_scheduler.empty(); }
};

#endif

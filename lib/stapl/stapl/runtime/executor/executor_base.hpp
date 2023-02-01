/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_EXECUTOR_EXECUTOR_BASE_HPP
#define STAPL_RUNTIME_EXECUTOR_EXECUTOR_BASE_HPP

#include "chunker.hpp"
#include "runnable_base.hpp"
#include "../constants.hpp"
#include "../exception.hpp"
#include "../tags.hpp"
#include "../utility/bool_mutex.hpp"
#include <stapl/utility/down_cast.hpp>
#include <iosfwd>
#include <type_traits>
#include <utility>
#include <boost/function.hpp>

namespace stapl {

std::size_t get_default_executor_window_size(void) noexcept;
std::size_t get_default_executor_retire_chunk(void) noexcept;

class executor_base;


namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Returns the executor associated with the location of gang with id
///        @p gid from the stack.
///
/// @warning If the location is not found, this function aborts execution. Right
///          now we only support nested parallel sections that have a parent on
///          the stack.
///
/// @ingroup runtimeMetadata
//////////////////////////////////////////////////////////////////////
executor_base& get_executor(const gang_id gid);

} // namespace runtime


//////////////////////////////////////////////////////////////////////
/// @brief Executor base class.
///
/// @see executor, gang_executor
/// @ingroup executors
//////////////////////////////////////////////////////////////////////
class executor_base
: public runnable_base
{
public:
  using result_type   = runnable_base::status_type;
private:
  // Using boost::function instead of std::function to avoid mallocs.
  using notifier_type = boost::function<void(void)>;

  chunker::entry_type* m_chunker_entry;
  notifier_type        m_runnable_notifier;
  notifier_type        m_finished_notifier;
protected:
  runtime::bool_mutex  m_executing;

public:
  executor_base(void) noexcept
  : m_chunker_entry(nullptr)
  { }

  ~executor_base(void) override;

  executor_base(executor_base const&) = delete;
  executor_base& operator=(executor_base const&) = delete;

protected:
  /////////////////////////////////////////////////////////////////////
  /// @brief Signals that the executor is runnable.
  //////////////////////////////////////////////////////////////////////
  void notify_runnable(void)
  {
    if (bool(m_runnable_notifier))
      m_runnable_notifier();
  }

  /////////////////////////////////////////////////////////////////////
  /// @brief Signals that the executor has finished.
  //////////////////////////////////////////////////////////////////////
  void notify_finished(void)
  {
    if (bool(m_finished_notifier))
      m_finished_notifier();
  }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc add_task(runnable_base*,none_t)
  //////////////////////////////////////////////////////////////////////
  virtual void add_task_impl(runnable_base*) = 0;

  //////////////////////////////////////////////////////////////////////
  /// @copydoc add_executor(executor_base*,none_t,const bool)
  //////////////////////////////////////////////////////////////////////
  virtual void add_executor_impl(executor_base*,
                                 const bool is_gang_executor) = 0;

  //////////////////////////////////////////////////////////////////////
  /// @brief Outputs information about this @ref executor_base.
  //////////////////////////////////////////////////////////////////////
  virtual void print(std::ostream&) = 0;

public:
  /////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if this executor is inserted to another executor.
  //////////////////////////////////////////////////////////////////////
  bool is_bound(void) const noexcept
  { return bool(m_runnable_notifier); }

  /////////////////////////////////////////////////////////////////////
  /// @brief Binds this executor to the executor of @p gid.
  //////////////////////////////////////////////////////////////////////
  void bind_to(const runtime::gang_id gid)
  {
    STAPL_RUNTIME_ASSERT(gid!=runtime::invalid_gang_id);
    STAPL_RUNTIME_ASSERT(!m_runnable_notifier &&
                         !m_finished_notifier &&
                         !m_chunker_entry);
    if (empty()) {
      m_runnable_notifier = [gid, this]
                            {
                              auto& parent_ex = runtime::get_executor(gid);
                              parent_ex.add_executor(this, true);
                            };
    }
    else {
      auto& parent_ex = runtime::get_executor(gid);
      parent_ex.add_executor(this, true);
    }
  }

  /////////////////////////////////////////////////////////////////////
  /// @brief Sets the notifiers to be called when the executor becomes runnable
  ///        or has finished execution and associates it with a chunker entry.
  //////////////////////////////////////////////////////////////////////
  template<typename RunnableFunction, typename FinishedFunction>
  void set_notifiers(RunnableFunction&& rf,
                     chunker::entry_type& entry,
                     FinishedFunction&& ff)
  {
    STAPL_RUNTIME_ASSERT(!m_finished_notifier && !m_chunker_entry);
    m_runnable_notifier = std::forward<RunnableFunction>(rf);
    m_finished_notifier = std::forward<FinishedFunction>(ff);
    m_chunker_entry     = &entry;
  }

  /////////////////////////////////////////////////////////////////////
  /// @brief Retires the chunker entry associated with this executor.
  //////////////////////////////////////////////////////////////////////
  void retire_chunker_entry(void) noexcept
  {
    if (!m_chunker_entry)
      return;
    m_chunker_entry->retire();
    m_chunker_entry = nullptr;
  }

  /////////////////////////////////////////////////////////////////////
  /// @brief Returns @c true if the executor is processing entries.
  //////////////////////////////////////////////////////////////////////
  bool idle(void) const noexcept
  { return !m_executing.is_locked(); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds a task without scheduling information.
  //////////////////////////////////////////////////////////////////////
  void add_task(runnable_base* t)
  { add_task_impl(t); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an executor with the given scheduling information.
  //////////////////////////////////////////////////////////////////////
  template<typename SchedInfo>
  void add_executor(executor_base*, SchedInfo&&,
                    const bool is_gang_executor = false);

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an executor without scheduling information.
  //////////////////////////////////////////////////////////////////////
  void add_executor(executor_base* ex, none_t,
                    const bool is_gang_executor = false)
  { add_executor_impl(ex, is_gang_executor); }

  //////////////////////////////////////////////////////////////////////
  /// @copydoc add_executor(executor_base*,none_t,const bool)
  //////////////////////////////////////////////////////////////////////
  void add_executor(executor_base* ex,
                    const bool is_gang_executor = false)
  { add_executor_impl(ex, is_gang_executor); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Populates the executor with entries.
  ///
  /// @return Status of the executor.
  //////////////////////////////////////////////////////////////////////
  virtual result_type populate(void) = 0;

  //////////////////////////////////////////////////////////////////////
  /// @brief Executes some entries.
  //////////////////////////////////////////////////////////////////////
  virtual result_type operator()(void) = 0;

  //////////////////////////////////////////////////////////////////////
  /// @brief Executes all entries.
  //////////////////////////////////////////////////////////////////////
  virtual void operator()(execute_all_t) = 0;

  virtual bool empty(void) = 0;

  friend std::ostream& operator<<(std::ostream&, executor_base&);
};


namespace runtime {

//////////////////////////////////////////////////////////////////////
/// @brief Intermediate class to allow adding entries with scheduling
///        information.
///
/// @tparam Scheduler Scheduler type.
///
/// This class is required because @ref executor objects are pointed to through
/// the @ref executor_base objects which are unaware of the scheduling
/// information.
///
/// @ingroup executorsImpl
//////////////////////////////////////////////////////////////////////
template<typename SchedInfo>
class executor_intermediate
  : public executor_base
{
protected:
  virtual void add_executor_impl(executor_base*,
                                 SchedInfo const&,
                                 const bool is_gang_executor) = 0;
  virtual void add_executor_impl(executor_base*,
                                 SchedInfo&&,
                                 const bool is_gang_executor) = 0;

  using executor_base::add_task_impl;
  using executor_base::add_executor_impl;

  friend class executor_base;
};

} // namespace runtime


// Implementation of executor_base::add_executor()
template<typename SchedInfo>
inline void executor_base::add_executor(executor_base* ex,
                                        SchedInfo&& sched_info,
                                        const bool is_gang_executor)
{
  using derived_type = runtime::executor_intermediate<
                         typename std::decay<SchedInfo>::type>;
  derived_type* e = down_cast<derived_type*>(this);
  e->add_executor_impl(ex,
                       std::forward<SchedInfo>(sched_info),
                       is_gang_executor);
}

} // namespace stapl

#endif

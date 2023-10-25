/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_RUNTIME_EXECUTOR_SCHEDULER_SCHED_HPP
#define STAPL_RUNTIME_EXECUTOR_SCHEDULER_SCHED_HPP

#include <map>
#include <utility>

#include <boost/intrusive/set.hpp>
#include <boost/intrusive/slist.hpp>

#include "default_info.hpp"
#include "sched_entry.hpp"
#include "task_placement.hpp"
#include "../../serialization_fwd.hpp"

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class that provides a copy constructor to
///        @c boost::intrusive::slist.
///
/// @ingroup executorsImpl
///
/// @todo Remove class by supporting correct move semantics for schedulers.
//////////////////////////////////////////////////////////////////////
template<typename Entry>
struct copyable_intrusive_list
  : public boost::intrusive::make_slist<
      Entry,
      boost::intrusive::cache_last<true>,
      boost::intrusive::constant_time_size<true>
    >::type
{
  copyable_intrusive_list(void) = default;

  copyable_intrusive_list(copyable_intrusive_list const& other)
  { STAPL_RUNTIME_ASSERT(other.empty()); }

  copyable_intrusive_list& operator=(copyable_intrusive_list const&) = delete;
};


//////////////////////////////////////////////////////////////////////
/// @brief FIFO scheduler.
///
/// @ingroup scheduling
//////////////////////////////////////////////////////////////////////
template<typename Placement>
class fifo_scheduler
  : public Placement
{
private:
  typedef boost::intrusive::slist_base_hook<>     hook_type;
public:
  typedef none_t                                  sched_info_type;
  typedef sched_entry<sched_info_type, hook_type> entry_type;
private:
  typedef copyable_intrusive_list<entry_type>     list_type;
public:
  typedef typename list_type::size_type           size_type;

private:
  list_type m_entries;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Generic constructor that forwards any arguments to the placement
  ///        policy. Allows arbitrary initialization of the defined placement
  ///        policy, as the scheduler itself needs no initialization parameters.
  //////////////////////////////////////////////////////////////////////
  template<typename ...Args>
  fifo_scheduler(Args&&... args)
    : Placement(std::forward<Args>(args)...)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an active entry to the scheduler.
  //////////////////////////////////////////////////////////////////////
  void add_active(entry_type& e) noexcept
  { m_entries.push_back(e); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an idle entry to the scheduler.
  //////////////////////////////////////////////////////////////////////
  void add_idle(entry_type&) noexcept
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the next entry to be executed.
  //////////////////////////////////////////////////////////////////////
  entry_type& next(void) noexcept
  {
    auto& e = m_entries.front();
    m_entries.pop_front();
    return e;
  }

  bool empty(void) const noexcept
  { return m_entries.empty(); }

  bool ready(void) const noexcept
  { return !empty(); }

  size_type size(void) const noexcept
  { return m_entries.size(); }

  void define_type(typer& t)
  {
    t.base<Placement>(*this);
    t.transient(m_entries);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Default scheduler is the @ref fifo_scheduler.
///
/// @ingroup scheduling
//////////////////////////////////////////////////////////////////////
typedef fifo_scheduler<default_task_placement> default_scheduler;


//////////////////////////////////////////////////////////////////////
/// @brief Default gang scheduler is the @ref fifo_scheduler.
///
/// @ingroup scheduling
//////////////////////////////////////////////////////////////////////
typedef fifo_scheduler<default_task_placement> default_gang_scheduler;


//////////////////////////////////////////////////////////////////////
/// @brief Persistent scheduler that extends from @ref default_scheduler.
///
/// @ingroup scheduling
//////////////////////////////////////////////////////////////////////
class persistent_scheduler
  : public default_scheduler
{
public:
  typedef void enable_persistence;
};


//////////////////////////////////////////////////////////////////////
/// @brief Priority scheduler that supports integer priority.
///
/// @tparam MAX_PRIORITY Maximum priority supported.
/// @tparam SchedInfo    Scheduling information object type.
///
/// @ingroup scheduling
//////////////////////////////////////////////////////////////////////
template<unsigned int MAX_PRIORITY = 10,
         typename SchedInfo        = default_info>
class priority_scheduler
  : public default_task_placement
{
private:
  typedef boost::intrusive::slist_base_hook<>     hook_type;
public:
  typedef SchedInfo                               sched_info_type;
  typedef sched_entry<sched_info_type, hook_type> entry_type;
private:
  typedef copyable_intrusive_list<entry_type>     list_type;
public:
  typedef typename list_type::size_type           size_type;

private:
  list_type    m_entries[MAX_PRIORITY];
  unsigned int m_current_priority;

public:
  priority_scheduler(void) noexcept
    : m_current_priority(0)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an active entry to the scheduler.
  //////////////////////////////////////////////////////////////////////
  void add_active(entry_type& e) noexcept
  {
    unsigned int priority = e.sched_info().priority();
    if (priority < m_current_priority)
      m_current_priority = priority;
    m_entries[priority].push_back(e);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an idle entry to the scheduler.
  //////////////////////////////////////////////////////////////////////
  void add_idle(entry_type&) noexcept
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the next entry to be executed.
  //////////////////////////////////////////////////////////////////////
  entry_type& next(void) noexcept
  {
    if (m_entries[m_current_priority].empty())
    {
      for (; m_current_priority != MAX_PRIORITY; ++m_current_priority)
        if (!m_entries[m_current_priority].empty())
          break;
    }

    // FIXME - is this fastest?
    //
    auto& e = *(m_entries[m_current_priority].begin());
    m_entries[m_current_priority].erase(m_entries[m_current_priority].begin());
    return e;
  }

  bool empty(void) const noexcept
  {
    for (unsigned int i = 0; i != MAX_PRIORITY; ++i) {
      if (!m_entries[i].empty())
        return false;
    }
    return true;
  }

  bool ready(void) const noexcept
  { return !empty(); }

  size_type size(void) const noexcept
  {
    size_type total_size = 0;
    for (unsigned int i = 0; i != MAX_PRIORITY; ++i)
      total_size += m_entries[i].size();
    return total_size;
   }
};


//////////////////////////////////////////////////////////////////////
/// @brief Arbitrary priority scheduler that supports any type of priority.
///
/// @tparam SchedInfo Scheduling information object type.
///
/// @ingroup scheduling
//////////////////////////////////////////////////////////////////////
template<typename SchedInfo = default_info>
class arbitrary_priority_scheduler
  : public default_task_placement
{
private:
  typedef boost::intrusive::slist_base_hook<>     hook_type;
public:
  typedef SchedInfo                               sched_info_type;
  typedef sched_entry<sched_info_type, hook_type> entry_type;
private:
  typedef typename sched_info_type::priority_type priority_type;
  typedef copyable_intrusive_list<entry_type>     list_type;
  typedef std::map<priority_type, list_type>      map_type;
public:
  typedef typename list_type::size_type           size_type;

private:
  map_type m_queues;

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an active entry to the scheduler.
  //////////////////////////////////////////////////////////////////////
  void add_active(entry_type& e) noexcept
  { m_queues[e.sched_info().priority()].push_back(e); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an idle entry to the scheduler.
  //////////////////////////////////////////////////////////////////////
  void add_idle(entry_type&) noexcept
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the next entry to be executed.
  ///
  /// @todo Optimize using @c find_first_set().
  //////////////////////////////////////////////////////////////////////
  entry_type& next(void) noexcept
  {
    // TODO optimize using find first set
    auto first = m_queues.begin();
    auto& q = (*first).second;
    auto& e = q.front();
    q.pop_front();

    if (q.empty())
      m_queues.erase(first);

    return e;
  }

  bool empty(void) const noexcept
  { return m_queues.empty(); }

  bool ready(void) const noexcept
  { return !empty(); }

  size_type size(void) const noexcept
  {
    size_type total_size = 0;
    for (auto&& i : m_queues)
      total_size += i->second.size();
    return total_size;
  }
};

} // namespace stapl

#endif

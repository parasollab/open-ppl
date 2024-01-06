/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cmath>
#include <iostream>

#include <boost/intrusive/slist.hpp>

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/array.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/utility/down_cast.hpp>
#include <stapl/runtime/executor/terminator.hpp>
#include <stapl/runtime/executor/scheduler/task_placement.hpp>
#include <stapl/runtime/executor/runnable_base.hpp>

//////////////////////////////////////////////////////////////////////
/// @brief Wrapper class that provides a copy constructor to
///        @c boost::intrusive::slist.
///
/// This class differs from the copyable_intrusive_list in sched.hpp by
/// actually performing a swap with the list passed instead of providing
/// the copy constructor as an empty function to appease the compiler. This
/// was necessary to support a dynamic number of priorities in the
/// strict_priority_scheduler.
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
  { this->swap(const_cast<copyable_intrusive_list&>(other)); }

  copyable_intrusive_list& operator=(copyable_intrusive_list const&) = delete;
};


class strict_priority_scheduler;


//////////////////////////////////////////////////////////////////////
/// @brief Specialized task placement for the strict_priority_scheduler
/// prevents tasks from being placed on locations other than the one
/// that spawned them.
///
/// struct holds a pointer to the scheduler so the counts of added and
/// executed tasks can be maintained.
//////////////////////////////////////////////////////////////////////
struct strict_priority_task_placement
{
private:
  strict_priority_scheduler* m_parent_scheduler;
  stapl::default_info        m_priority;

public:
  using sched_info_type = stapl::default_info;

  strict_priority_task_placement(stapl::default_info priority =
    stapl::default_info(0));

  template<typename WF, typename View, typename ...Views>
  typename std::enable_if<!stapl::is_factory<WF>::value,
             stapl::locality_info>::type
  execution_location(WF const&, View const& view, Views const&... views);

  stapl::default_info get_sched_info(void) const noexcept
  { return m_priority; }

  void notify_finished(void) noexcept;
};


//////////////////////////////////////////////////////////////////////
/// @brief Priority scheduler that supports integer priority.
///
/// Scheduler will process all top priority work before moving to the next
/// priority. This is required to get optimal sweep scheduling.
//////////////////////////////////////////////////////////////////////
class strict_priority_scheduler
  : public strict_priority_task_placement
{
private:
  using hook_type = boost::intrusive::slist_base_hook<>;

public:
  using sched_info_type = stapl::default_info;
  using entry_type = stapl::sched_entry<sched_info_type, hook_type>;

private:
  using list_type = copyable_intrusive_list<entry_type>;

public:
  using size_type = typename list_type::size_type;

private:
  using status_type = stapl::runnable_base::status_type;

  // size_t used to keep track of tasks remaining to execute at the priority
  using priority_list = std::pair<std::size_t, list_type>;

  std::vector<priority_list>  m_entries;
  unsigned int                m_current_priority;
  stapl::location_type        m_lid;

  priority_list& update_priority(unsigned int priority)
  {
    if (priority < m_current_priority)
      m_current_priority = priority;
    if (priority >= m_entries.size())
      m_entries.resize(priority+1);
    return m_entries[priority];
  }

  void skip_empty(void)
  {
    if (m_current_priority == m_entries.size())
      m_current_priority = 0;
    for (; m_current_priority != m_entries.size(); ++m_current_priority)
      if (m_entries[m_current_priority].first != 0 ||
          !m_entries[m_current_priority].second.empty())
        break;
  }

public:
  strict_priority_scheduler(void) noexcept
    : m_entries(300), m_current_priority(0), m_lid(stapl::get_location_id())
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Increment the task count for the specified priority
  ///
  /// Called from task_placement as a task is being added.
  //////////////////////////////////////////////////////////////////////
  void increment(unsigned int priority)
  {
    priority_list& pl = update_priority(priority);
    ++pl.first;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Decrement the task count for the specified priority
  ///
  /// Called from task_placement as a task is executed.
  //////////////////////////////////////////////////////////////////////
  void decrement(unsigned int priority)
  {
    stapl_assert(priority == m_current_priority,
                 "calling decrement on non-current priority");
    if (--m_entries[m_current_priority].first == 0)
      skip_empty();
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an active entry to the scheduler.
  //////////////////////////////////////////////////////////////////////
  void add_active(entry_type& e) noexcept
  { update_priority(e.sched_info().priority()).second.push_back(e); }

  //////////////////////////////////////////////////////////////////////
  /// @brief Adds an idle entry to the scheduler.
  //////////////////////////////////////////////////////////////////////
  void add_idle(entry_type& e) noexcept
  { }


  //////////////////////////////////////////////////////////////////////
  /// @brief Returns the next entry to be executed.
  //////////////////////////////////////////////////////////////////////
  entry_type& next(void) noexcept
  {
    STAPL_RUNTIME_ASSERT(!empty());
    skip_empty();

    // FIXME - is this fastest?
    auto& e = *(m_entries[m_current_priority].second.begin());
    m_entries[m_current_priority].second.erase(
      m_entries[m_current_priority].second.begin());
    return e;
  }

  bool ready(void) noexcept
  {
    skip_empty();
    return m_current_priority != m_entries.size() &&
           !m_entries[m_current_priority].second.empty();
  }

  bool empty(void) const noexcept
  {
    for (unsigned int i = 0; i != m_entries.size(); ++i) {
      if (m_entries[i].first != 0 || !m_entries[i].second.empty())
        return false;
    }
    return true;
  }

  size_type size(void) const noexcept
  {
    size_type total_size = 0;
    for (unsigned int i = 0; i != m_entries.size(); ++i)
      total_size += m_entries[i].first != 0 ? m_entries[i].first :
        m_entries[i].second.size();
    return total_size;
  }
};

strict_priority_task_placement::strict_priority_task_placement(
  stapl::default_info priority)
  : m_parent_scheduler(nullptr),
    m_priority(priority)
{ }


template<typename WF, typename View, typename ...Views>
typename std::enable_if<!stapl::is_factory<WF>::value,
           stapl::locality_info>::type
strict_priority_task_placement::execution_location(
  WF const&, View const& view, Views const&... views)
{
#ifndef STAPL_NDEBUG
  const auto static_vote_result = stapl::count_static_votes(view, views...);

  stapl::locality_info info = stapl::default_task_placement_impl::apply(
    stapl::get<0>(static_vote_result), stapl::get<1>(static_vote_result));

  stapl_assert(info.location() == stapl::get_location_id() &&
               info.qualifier() == stapl::LQ_CERTAIN,
               "strict_priority_task_placement returned non-local result");
#endif
  if (m_parent_scheduler == nullptr)
    m_parent_scheduler =
      &stapl::down_cast<stapl::gang_executor<strict_priority_scheduler>*>
        (&stapl::get_executor())->get_scheduler();
  m_parent_scheduler->increment(m_priority.priority());
  return stapl::locality_info(stapl::LQ_CERTAIN, stapl::get_affinity(),
                              stapl::rmi_handle::reference(),
                              stapl::get_location_id());
}


void strict_priority_task_placement::notify_finished(void) noexcept
{
  if (m_parent_scheduler == nullptr)
    m_parent_scheduler =
      &stapl::down_cast<stapl::gang_executor<strict_priority_scheduler>*>
        (&stapl::get_executor())->get_scheduler();
  m_parent_scheduler->decrement(m_priority.priority());
}


struct increment_wf
{
  typedef int result_type;

  // Function operator invoked by the first task in each chain
  template<typename Element>
  result_type operator()(Element&& e) const
  {
    e += 1;
    return 1;
  }

  // Function operator invoked by all other tasks in a chain
  template<typename Element, typename Incoming>
  result_type operator()(Element&& e, Incoming&& i) const
  {
    e += i;
    return 1;
  }
};


// Factory that constructs the specified number of chains of tasks, one task
// per location.
struct directed_chains_factory
  : public stapl::task_factory_base
{
private:
  int m_num_chains;
  bool m_forward;

public:
  using result_type = void;

  directed_chains_factory(int num_chains, bool forward)
    : m_num_chains(num_chains), m_forward(forward)
  { }

  template<typename TGV, typename View>
  void operator()(TGV const& tgv, View& input_view)
  {
    stapl::location_type lid  = input_view.get_location_id();
    stapl::location_type locs = input_view.get_num_locations();

    stapl::location_type start_lid = m_forward ? 0 : locs-1;
    stapl::location_type last_lid  = m_forward ? locs-1 : 0;

    // local element of the input_view
    auto array_element_spec = std::make_pair(&input_view, lid);

    size_t tid = lid;
    size_t pred_tid = m_forward ? tid-1 : tid+1;
    for (int i = 0; i != m_num_chains; ++i)
    {
      if (lid == start_lid)
      {
        // First task in the chain has no predecessor,
        // and in the single location task has no successor.
        int num_succs = locs != 1 ? 1 : 0;
        tgv.add_task(tid, increment_wf(), num_succs, array_element_spec);
      }
      else
      {
        // Last task in the chain has no successors, others have one
        int num_succs =  lid == last_lid ? 0 : 1;
        tgv.add_task(tid, increment_wf(), num_succs,
          array_element_spec, stapl::consume<int>(tgv, pred_tid));
      }
      tid += locs;
      pred_tid += locs;
    }

    this->m_finished = true;
  }

  void reset()
  {
    this->m_finished = false;
  }
};


stapl::exit_code stapl_main(int argc, char** argv)
{
  stapl::set_executor_scheduler(strict_priority_scheduler());

  stapl::do_once([](void) {
    std::cout << "Testing paragraph with strict_priority_scheduler...";
  });

  const int n_chains = atoi(argv[1]);
  const stapl::location_type n_locs = stapl::get_num_locations();
  const stapl::location_type lid = stapl::get_location_id();
  stapl::array<int> a(n_locs, 0);
  stapl::array_view<stapl::array<int>> av(a);

  // higher values are lower priority with the strict_priority_scheduler.
  stapl::default_info decreasing_priority(lid);
  stapl::default_info increasing_priority(n_locs-1-lid);

  // backward chain that will start on last location
  stapl::make_paragraph(
    directed_chains_factory(n_chains, false),
    stapl::fifo_scheduler<strict_priority_task_placement>(increasing_priority),
    av)(0);

  // forward chain that will start of location 0
  stapl::make_paragraph(
    directed_chains_factory(n_chains, true),
    stapl::fifo_scheduler<strict_priority_task_placement>(decreasing_priority),
    av)(0);

  stapl::get_executor()(stapl::execute_all);

  stapl::location_type sum = stapl::accumulate(av, 0);

  stapl::do_once([=](void) {
   if (sum == 2*n_chains*n_locs)
     std::cout << "Passed\n";
    else
     std::cout << "Failed\n";
  });

  return EXIT_SUCCESS;
}

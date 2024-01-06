/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <string>
#include <vector>

#include <boost/intrusive/slist.hpp>

#include "paragraph_test_patterns.hpp"

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include <stapl/runtime/executor/scheduler/work_stealing_scheduler.hpp>


namespace detail {

struct sched_info_t
: public stapl::work_stealing_sched_info
{
  typedef void         enable_migration;
  typedef unsigned int priority_type;

  sched_info_t(stapl::none_t = stapl::none)
  { }

  priority_type priority(void) const
  {
    stapl::abort("priority unexpectedly checked");
    return 0;
  }
};

} // namespace detail


class task_migrating_scheduler
  : public stapl::p_object,
    public stapl::default_task_placement
{
private:
  typedef boost::intrusive::slist_base_hook<>        hook_type;
public:
  typedef detail::sched_info_t                       sched_info_type;
  typedef stapl::work_stealing_sched_entry<
            sched_info_type, hook_type
          >                                          entry_type;
private:
  typedef stapl::copyable_intrusive_list<entry_type> list_t;

  list_t                                m_entries;
  size_t                                m_num_migrations;
  stapl::task_graph*                    m_tg_ptr;

  task_migrating_scheduler& operator=(task_migrating_scheduler const&);

public:
  task_migrating_scheduler(void)
   : m_num_migrations(0), m_tg_ptr(nullptr)
  {
    /*m_entries.reserve(...);*/
  }

  task_migrating_scheduler(task_migrating_scheduler const& other)
    : m_entries(other.m_entries),
      m_num_migrations(0),
      m_tg_ptr(other.m_tg_ptr)
  {
    stapl_assert(other.m_entries.empty(), "other.m_entries not empty");
    stapl_assert(other.m_num_migrations == 0, "other.m_num_migrations != zero");
  }

  void set_tg(stapl::task_graph& tg)
  {
    m_tg_ptr = &tg;
  }

public:
  void add_active(entry_type& e)
  {
    m_entries.push_back(e);
  }

  void add_idle(entry_type&) noexcept
  { }

  entry_type& next(void)
  {
    stapl_assert(!m_entries.empty(), "next called on empty scheduler");

    entry_type& e = m_entries.front();
    m_entries.pop_front();

    return e;
  }

  bool empty(void) //const
  {
    // try to migrate some random number of the remaining
    // entries (if any) to some random locations
    migrate_entries();

    return m_entries.empty();
  }

  bool ready(void) //const
  {
    // try to migrate some random number of the remaining
    // entries (if any) to some random locations
    migrate_entries();

    return !m_entries.empty();
  }

  size_t size(void) const
  {
    return m_entries.size();
  }

  void migrate_entries(void)
  {
    // to prevent infinite migration, bound
    // the total number of migrations
    size_t MAX_MIGRATIONS = 1000;

    if (m_entries.empty() || m_num_migrations > MAX_MIGRATIONS)
      return;

    // randomly try to migrate entries to another location
#if 1
    int num_to_migrate = m_entries.size();
#else
    int num_entries = m_entries.size();
    int num_to_migrate = rand()%num_entries;

    if (!num_to_migrate)
      num_to_migrate = num_entries;

#endif
    while (num_to_migrate--)
    {
      entry_type& e = m_entries.front();
      m_entries.pop_front();

      stapl_assert(!e.is_linked(), "Boom");

      if (e.is_migratable())
      {
        const size_t dest = rand() % this->get_num_locations();

        if (dest == this->get_location_id())
        {
          m_entries.push_front(e);
        }
        else
        {
      stapl_assert(!e.is_linked(), "Boom2");
          e.migrate(dest, m_tg_ptr);
          ++m_num_migrations;
        }
      }
      else
      {
        // push back the non migratable entry to the end. Really for this
        // particular test case this case should only occur once.  Only the
        // factory task/other executors are non-migratable. In this test
        // case there are no nested executors. Also without incremental
        // generation when the factory task is in the runqueue, it is the
        // only task that is in the runqueue.
        m_entries.push_back(e);
      }
    }
  }
};


template<typename Pattern, typename ViewIn>
struct compute_result_wf
{
  Pattern   m_pattern;
  ViewIn*   m_vw_in;

  typedef void result_type;

  compute_result_wf(Pattern const& pattern, ViewIn& vw_in)
    : m_pattern(pattern),
      m_vw_in(&vw_in)
  { }

 template<typename Reference1, typename Reference2>
 //void operator()(Reference1 const& tid, Reference2 const& result_val) const
 //void operator()(Reference1 const& tid, Reference2 & result_val) const
 void operator()(Reference1 tid, Reference2 result_val) const
 {
   result_val = (*m_vw_in)[tid];
   // get the pred list for tid
   std::vector<size_t> pred_list = m_pattern.get_pred_list(tid);
   for (unsigned int j=0; j<pred_list.size(); ++j)
     result_val = result_val + (*m_vw_in)[pred_list[j]];
 }

  void define_type(stapl::typer& t)
  {
    t.member(m_pattern);
    t.member(m_vw_in);
  }
};


template<typename ViewIn, typename ViewOut>
struct add_wf
{
  typedef int result_type;

  ViewIn*   m_vw_in;
  ViewOut*  m_vw_out;
  size_t    m_tid;

  add_wf(ViewIn& vw_in, ViewOut& vw_out, size_t tid)
    : m_vw_in(&vw_in),
      m_vw_out(&vw_out),
      m_tid(tid)
  { }

  // no preds
  result_type operator()() const
  {
    (*m_vw_out)[m_tid] = (*m_vw_in)[m_tid];
    return (*m_vw_in)[m_tid];
  }

  // one pred
  template<typename Reference>
  result_type operator()(Reference const& ref) const
  {
    (*m_vw_out)[m_tid] = (*m_vw_in)[m_tid] + ref;
    return (*m_vw_in)[m_tid];
  }

  // two preds
  template<typename Reference1, typename Reference2>
  result_type operator()(Reference1 const& ref1, Reference2 const& ref2) const
  {
    (*m_vw_out)[m_tid] = (*m_vw_in)[m_tid] + ref1 + ref2;
    return (*m_vw_in)[m_tid];
  }

  // three preds
  template<typename Reference1, typename Reference2, typename Reference3>
  result_type operator()(Reference1 const& ref1,
                         Reference2 const& ref2,
                         Reference3 const& ref3) const
  {
    (*m_vw_out)[m_tid] = (*m_vw_in)[m_tid] + ref1 + ref2 + ref3;
    return (*m_vw_in)[m_tid];
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_vw_in);
    t.member(m_vw_out);
    t.member(m_tid);
  }
};


template<typename Scheduler, bool EnableMigration = true>
struct pattern_tester
{
  int             m_tasks_per_call;
  size_t          m_sleep_time;
  const Scheduler m_sched;

  pattern_tester(int tasks_per_call, size_t sleep_time,
                 Scheduler const& sched = Scheduler())
    : m_tasks_per_call(tasks_per_call),
      m_sleep_time(sleep_time),
      m_sched(sched)
  { }

  typedef typename Scheduler::sched_info_type sched_info_type;

  template<typename Pattern>
  void test_pattern(Pattern const& pattern, std::string name) const
  {
    // test for signals with viewless factory
    typedef viewless_pattern_factory_t<Pattern, sched_info_type> factory_t;
    typedef stapl::paragraph<Scheduler, factory_t>            tg_t;

    // execute a blocking paragraph with the given pattern
    if (stapl::get_location_id() == 0)
      std::cout << "Testing Pattern "
                << name << " for signals ...";

    tg_t(factory_t(pattern, m_tasks_per_call, m_sleep_time),
         m_sched, EnableMigration)();

    // if we are here, then it means that the test passed.
    if (stapl::get_location_id() == 0) std::cout << "Passed" << std::endl;


    // test for data flow of values
    if (stapl::get_location_id() == 0)
      std::cout << "Testing Pattern "
                << name << " for value consumption ...";

    typedef stapl::array<int>        cnt_t;
    typedef stapl::array_view<cnt_t> view_t;

    const int num_elems = pattern.num_elems();

    cnt_t  ct_in(num_elems);
    cnt_t  ct_out(num_elems);
    view_t vw_in(ct_in);
    view_t vw_out(ct_out);

    stapl::iota(vw_in, 0);
    stapl::fill(vw_out, 0);

    // prepare the result container and view
    // each task with tid returns value vw_in[tid]
    // each task outputs vw_out[tid] =
    //   vw_in[tid] + vw_in[pred0] + vw_in[pred1] + vw_in[pred2]
    // a maximum of three predecessors are supported for this test
    cnt_t  ct_result(num_elems);
    view_t vw_result(ct_result);

    stapl::map_func(compute_result_wf<Pattern, view_t>(pattern, vw_in),
                    stapl::counting_view<int>(num_elems, 0), vw_result);

    // test the pattern
    typedef pattern_factory_t<
      Pattern, add_wf<view_t, view_t>, view_t, view_t, sched_info_type
    >                                                           val_factory_t;
    typedef stapl::paragraph<Scheduler, val_factory_t>            val_tg_t;

    val_tg_t(
      val_factory_t(pattern, m_tasks_per_call, m_sleep_time, vw_in, vw_out),
      m_sched, EnableMigration
    )();

    // compare the result and the output view
    bool result = stapl::equal(vw_out, vw_result);
    if (result)
    {
      if (stapl::get_location_id() == 0) std::cout << "Passed" << std::endl;
    }
    else
    {
      if (stapl::get_location_id() == 0) std::cout << "Failed" << std::endl;
    }
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using std::cout; using std::endl;

  if (stapl::get_location_id() == 0)
  {
    cout << "Testing paragraph for task migration on "
         << stapl::get_num_locations() << " locations...";

    cout << endl;
  }

  int rows           = 4;
  int cols_per_proc  = 1;
  int tasks_per_call = 0;
  size_t sleep_time  = 0;

  if (argc>=2)
    rows = atoi(argv[1]);

  if (argc>=3)
    tasks_per_call = atoi(argv[2]);

  if (argc>=4)
    sleep_time     = atoi(argv[3]);

  if (argc>=5)
  {
    stapl_assert(false, "varying cols_per_proc parameter not supported yet");
    cols_per_proc         = atoi(argv[4]);
  }

  int cols           = cols_per_proc * stapl::get_num_locations();

  // create a pattern tester to test different patterns
  pattern_tester<task_migrating_scheduler> tester(tasks_per_call, sleep_time);
  //pattern_tester<stapl::default_scheduler> tester(tasks_per_call, sleep_time);

  // test basic patterns
  typedef basic_pattern_one<>   p_one_t;
  typedef basic_pattern_two<>   p_two_t;
  typedef basic_pattern_three<> p_three_t;
  typedef basic_pattern_four<>  p_four_t;
  typedef basic_pattern_five<>  p_five_t;

  tester.test_pattern(p_one_t(rows, cols), "basic_pattern_one");
  tester.test_pattern(p_two_t(rows, cols), "basic_pattern_two");
  tester.test_pattern(p_three_t(rows, cols), "basic_pattern_three");
  tester.test_pattern(p_four_t(rows, cols), "basic_pattern_four");
  tester.test_pattern(p_five_t(rows, cols), "basic_pattern_five");

  // test composed patterns
  //  COMPOSED_PATTERN_ONE
  //  <-- cols -->
  //  O-->O-->O-->O   |
  //  |   |   |   |   |
  //  v   v   v   v   |
  //  O-->O-->O-->O   | rows
  //  |   |   |   |   |
  //  v   v   v   v   |
  //  O-->O-->O-->O   v
  tester.test_pattern(composed_pattern<p_two_t, p_three_t>
                        (p_two_t(rows, cols), p_three_t(rows, cols)),
                      "composed_pattern_one");

  //  COMPOSED_PATTERN_TWO
  //  <-- cols -->
  //  O   O   O   O   |
  //  | \ | \ | \ |   |
  //  |  \|  \|  \|   |
  //  v   v   v   v   | rows
  //  O   O   O   O   |
  //  | \ | \ | \ |   |
  //  |  \|  \|  \|   |
  //  v   v   v   v   |
  //  O   O   O   O   v
  tester.test_pattern(composed_pattern<p_three_t, p_four_t>
                        (p_three_t(rows, cols), p_four_t(rows, cols)),
                      "composed_pattern_two");

  //  COMPOSED_PATTERN_THREE
  //  O-->O-->O-->O   |
  //  | \ | \ | \ |   |
  //  |  \|  \|  \|   |
  //  v   v   v   v   | rows
  //  O-->O-->O-->O   |
  //  | \ | \ | \ |   |
  //  |  \|  \|  \|   |
  //  v   v   v   v   |
  //  O-->O-->O-->O   v
  typedef composed_pattern<p_three_t, p_four_t> cp_three_four_t;
  cp_three_four_t cp_three_four(p_three_t(rows, cols), p_four_t(rows, cols));
  tester.test_pattern(composed_pattern<p_two_t, cp_three_four_t>
                        (p_two_t(rows, cols), cp_three_four),
                      "composed_pattern_three");

  //  COMPOSED_PATTERN_FOUR
  //  O  O  O  O  O   |
  //  |\/|\/|\/|\/|   |
  //  |/\|/\|/\|/\|   |
  //  v  v  v  v  v   | rows
  //  O  O  O  O  O   |
  //  |\/|\/|\/|\/|   |
  //  |/\|/\|/\|/\|   |
  //  v  v  v  v  v   |
  //  O  O  O  O  O   v
  tester.test_pattern(composed_pattern<cp_three_four_t, p_five_t>
                        (cp_three_four, p_five_t(rows, cols)),
                      "composed_pattern_four");

  return EXIT_SUCCESS;
}

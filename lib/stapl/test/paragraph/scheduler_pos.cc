/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/paragraph/paragraph.hpp>
#include <stapl/skeletons/utility/view_index_partition.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/containers/partitions/blocked_partition.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/skeletons/explicit/coarse_map_wf.h>
#include <stapl/skeletons/serial.hpp>

struct read_values_wf
{
  typedef void result_type;

  template<typename Reference>
  result_type operator()(Reference elem) const
  {
    std::cin >> elem;
  }
};


namespace stapl {

struct default_sched_wf
{
private:
  size_t           m_tasks_per_loc;
  location_type    m_loc_id;

public:
  typedef void result_type;

  default_sched_wf(size_t tasks_per_loc, location_type loc_id)
    : m_tasks_per_loc(tasks_per_loc),
      m_loc_id(loc_id)
  { }

  void define_type(stapl::typer& t)
  {
    t.member(m_tasks_per_loc);
    t.member(m_loc_id);
  }

  template<typename View0, typename View1, typename View2>
  result_type operator()(View0 const& view0,
                         View1 const& view1,
                         View2 const& view2) const
  {
    // calculate begin and end and current_index
    size_t current_index = view2;
    size_t begin         = m_tasks_per_loc * m_loc_id;
    size_t end           = begin + m_tasks_per_loc;

    stapl_assert(current_index>=begin && current_index<=end,
                 "wrong value of counting view encountered");
    stapl_assert(view1.size()>=end, "wrong view size encountered");

    // all tasks added before the current task should have been executed
    for (size_t i = begin; i<current_index; i++)
      stapl_assert(view1[i]==size_t(1),
                   "task not scheduled according to default policy");

    // all tasks added after the current task should not have been executed
    for (size_t i = current_index+1; i<end; i++)
      stapl_assert(view1[i]==size_t(0),
                   "task not scheduled according to default policy");

    // set the value for this invocation
    view1[current_index] = 1;
  }
};


struct priority_sched_wf
{
private:
  size_t m_tasks_per_loc;
  location_type    m_loc_id;

public:
  typedef void result_type;

  priority_sched_wf(size_t tasks_per_loc, location_type loc_id)
    : m_tasks_per_loc(tasks_per_loc),
      m_loc_id(loc_id)
  { }

  void define_type(stapl::typer& t)
  {
    t.member(m_tasks_per_loc);
    t.member(m_loc_id);
  }

  template<typename View0, typename View1, typename View2>
  result_type operator()(View0 const& view0, View1 const& view1,
                         View2 const& view2) const {
    // calculate begin and end and current_index
    size_t current_index = view2;
    size_t begin         = m_tasks_per_loc * m_loc_id;
    size_t end           = begin + m_tasks_per_loc;

    stapl_assert(current_index>=begin && current_index<=end,
                 "wrong value of counting view encountered");
    stapl_assert(view1.size()>=end, "wrong view size encountered");

    // all tasks of priority higher than the current task should
    // have been executed
    for (size_t i = current_index+1; i<end; i++)
      stapl_assert(view1[i]==size_t(1),
                   "task not scheduled according to priority");

    // all tasks of priority lower than the current task should
    // not have been executed
    for (size_t i = begin; i<current_index; i++)
      stapl_assert(view1[i]==size_t(0),
                   "task not scheduled according to priority");

    // set the value for this invocation
    view1[current_index] = 1;
  }
};


template <typename WF, typename SchedInfo>
struct sched_map_factory
  : public task_factory_base,
    private WF
{
public:
  using result_type = void;
  using coarsener_type = default_coarsener;

  sched_map_factory(WF const& wf)
    : task_factory_base(true),
      WF(wf) { }

  coarsener_type get_coarsener() const
  { return coarsener_type(); }

  void reset(void)
  {
    this->reset_view_indices();
    this->m_finished = false;
  }

  template<typename TGV, typename View0, typename View1, typename View2>
  void operator()(TGV const& tgv, View0 & view0, View1 & view1, View2 & view2);

private:
  template<typename TGV, typename FinalWF, typename View0,
           typename View1, typename View2>
  void add_task_helper(int, TGV const&, FinalWF const&, View0 const&,
                       View1 const&, View2 const&);
}; // class sched_map_factory


// for schedulers with no_sched_info
template <>
template <typename TGV, typename FinalWF, typename View0,
          typename View1, typename View2>
void
sched_map_factory<default_sched_wf, none_t>
  ::add_task_helper(int, TGV const& tgv, FinalWF const& wf, View0 const& view0,
                    View1 const& view1, View2 const& view2)
{
  tgv.add_task(none, wf, view0, view1, view2);
}


// for schedulers with default_info
template <>
template <typename TGV, typename FinalWF, typename View0,
          typename View1, typename View2>
void
sched_map_factory<priority_sched_wf, default_info>::
add_task_helper(int priority, TGV const& tgv, FinalWF const& wf,
                    View0 const& view0, View1 const& view1, View2 const& view2)
{
  tgv.add_task(default_info(priority), wf, view0, view1, view2);
}


template <typename WF, typename SchedInfo>
template <typename TGV, typename View0, typename View1, typename View2>
void sched_map_factory<WF, SchedInfo>::
operator()(TGV const& tgv, View0 & view0, View1 & view1, View2 & view2)
{
  // typedefs for view_index_iterator types
  typedef view_index_iterator<View0> id_iter_type0;
  typedef view_index_iterator<View1> id_iter_type1;
  typedef view_index_iterator<View2> id_iter_type2;

  // if this is the first invocation of operator(), do initialization
  // since this factory is only called once, this call is redundant
  if (!this->initialized())
  {
    tuple<id_iter_type0, id_iter_type1, id_iter_type2> id_it =
      partition_id_set(view0, view1, view2);

    //store type erased id_iterators in container of base class
    this->set_view_index_iterator(0, new id_iter_type0(get<0>(id_it)));
    this->set_view_index_iterator(1, new id_iter_type1(get<1>(id_it)));
    this->set_view_index_iterator(2, new id_iter_type2(get<2>(id_it)));
  }

  id_iter_type0 * iter0 =
    static_cast<id_iter_type0*>(this->get_view_index_iterator(0));

  id_iter_type1 * iter1 =
    static_cast<id_iter_type1*>(this->get_view_index_iterator(1));

  id_iter_type2 * iter2 =
    static_cast<id_iter_type2*>(this->get_view_index_iterator(2));

  // create the set of map tasks
  // add tasks with decreasing priority numbers (increasing priority) to
  // differentiate
  // between the behaviour of the default scheduler, the priority scheduler
  // and the
  // arbitrary priority scheduler
  int min_priority = iter0->size();
  for (; !iter0->at_end(); ++(*iter0), ++(*iter1), ++(*iter2))
  {
    add_task_helper(
      min_priority--, tgv,
      choose_wf<View0>()(
        static_cast<WF const&>(*this), coarse_map_wf<WF>(*this)
      ),
      std::make_pair(&view0, **iter0),
      std::make_pair(&view1, **iter1),
      std::make_pair(&view2, **iter2));
  }

  this->m_finished = true;
}

} // namespace stapl


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using std::cout;
  using std::endl;
  using namespace stapl;

  if (get_location_id() == 0)
    cout << "paragraph scheduler test on " << get_num_locations()
         << " locations..." << endl;

  typedef counter<default_timer> counter_type;
  counter_type c;

  typedef indexed_domain<size_t>                                   dom_t;
  typedef block_partitioner<dom_t>                                 part_t;
  typedef array<size_t, part_t>                                    cnt_t;
  typedef array_view<cnt_t>                                        vw_t;

  unsigned int block_size            = 1;
  unsigned int tasks_per_loc         = 10;
  unsigned int num_locs              = get_num_locations();
  unsigned int num_elems             = tasks_per_loc*num_locs;
  const unsigned int priority_levels = 50;

  stapl_assert(num_elems<priority_levels,
               "This test case assumes that the total number of elements \
                is less than the total number of allowed priority levels");

  cnt_t ct(part_t(dom_t(0, num_elems-1), block_size));
  vw_t vw1(ct);
  auto rep_view = make_repeat_view(vw1);

  typedef decltype(rep_view) rep_view_t;

  //
  // Default Scheduler
  //
  if (get_location_id()==0)
    cout << "Testing default scheduler...";

  fill(vw1, 0);

  typedef sched_map_factory<
    default_sched_wf, none_t>                        factory_def_t;

  typedef decltype(counting_view<size_t>(num_elems)) count_view_t;

  typedef stapl::paragraph<
    default_scheduler,
    factory_def_t, vw_t, rep_view_t, count_view_t> tg_def_t;

  tg_def_t(
    factory_def_t(default_sched_wf(tasks_per_loc, stapl::get_location_id())),
    vw1,
    rep_view,
    counting_view<size_t>(num_elems)
  )();

  if (get_location_id()==0)
    cout << "Passed" << endl;

  //
  // priority Scheduler
  //
  if (get_location_id()==0)
    cout << "Testing priority scheduler...";

  fill(vw1, 0);

  typedef sched_map_factory<
    priority_sched_wf, default_info>                factory_prio_t;

  typedef stapl::paragraph<
    priority_scheduler<priority_levels, default_info>,
    factory_prio_t, vw_t, rep_view_t, count_view_t> tg_prio_t;

  tg_prio_t(
    factory_prio_t(priority_sched_wf(tasks_per_loc, stapl::get_location_id())),
    vw1,
    rep_view,
    counting_view<size_t>(num_elems),
    priority_scheduler<priority_levels, default_info>()
  )();

  if (get_location_id()==0)
    cout << "Passed" << endl;

  //
  // Arbitrary Priority Scheduler
  //
  if (get_location_id()==0)
    cout << "Testing arbitrary priority scheduler...";

  fill(vw1, 0);

  typedef stapl::paragraph<
    arbitrary_priority_scheduler<>,
    factory_prio_t, vw_t, rep_view_t, count_view_t> tg_arb_prio_t;

  tg_arb_prio_t(
    factory_prio_t(priority_sched_wf(tasks_per_loc, stapl::get_location_id())),
    vw1,
    rep_view,
    counting_view<size_t>(num_elems)
  )();

  if (get_location_id()==0)
    cout << "Passed" << endl;

 //
 // serial_io (using fifo_scheduler with location 0 task placement
 //
 if (get_location_id()==0)
    cout << "Testing serial_io (location_0_placement_policy)...";

 array<int>                 s_ar(16);
 array_view<array<int> >    s_vw(s_ar);

 serial_io(read_values_wf(), s_vw);

 const int  expected_sum = accumulate(s_vw, 0);
 const int  actual_sum   = s_ar.size()*(s_ar.size()+1) / 2;

 if (get_location_id()==0)
 {
   if (expected_sum == actual_sum)
     cout << "Passed" << endl;
   else
     cout << "Failed" << endl;
 }

 return EXIT_SUCCESS;
}

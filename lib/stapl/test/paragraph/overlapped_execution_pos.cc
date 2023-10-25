/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <vector>

#include <stapl/paragraph/paragraph.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/runtime.hpp>


//        <--- length--->
//        <------- total_length -------->
//        <-----P0------> <-----P1------>

//        O--->O--->O--->O--->O--->O--->O   |
//        |    |    |    |    |    |    |   |
//        v    v    v    v    v    v    v   |
//        O--->O--->O--->O--->O--->O--->O   |
//        |    |    |    |    |    |    |   |
//        v    v    v    v    v    v    v   |
//        O--->O--->O--->O--->O--->O--->O   | breadth (and not total breadth :))
//        |    |    |    |    |    |    |   |
//        v    v    v    v    v    v    v   |
//        O--->O--->O--->O--->O--->O--->O   |
//        |    |    |    |    |    |    |   |
//        v    v    v    v    v    v    v   |
//        O--->O--->O--->O--->O--->O--->O   v


class id_calculator
{
  int length;
  int breadth;
  int procs;
  int num_elems;
  int total_length;

  typedef std::vector<int> vec_t;

public:
  id_calculator (int v_length, int v_breadth, int v_procs) {
    length       = v_length;
    breadth      = v_breadth;
    procs        = v_procs;
    total_length = length*procs;
    num_elems    = total_length*breadth;
  }

  std::vector<int> get_task_ids(int pid)
  {
    stapl_assert( ((pid>=0) && (pid<procs)), "received invalid proc id!" );
    vec_t vec;
    for (int y = 0; y < breadth; y++) {
      for (int x = 0; x < length; x++) {
        int tid = (y*total_length) + (pid*length) + x;
        vec.push_back(tid);
      }
    }
    return vec;
  }

  int get_left_pred(int tid) {
    stapl_assert( ((tid>=0)&&(tid<num_elems)), "received invalid task id!" );
    if ( (tid%total_length) == 0 )
      return -1;
    else
      return tid-1;
  }

  int get_top_pred(int tid) {
    stapl_assert( ((tid>=0)&&(tid<num_elems)), "received invalid task id!" );
    if ( (tid/total_length) == 0 )
      return -1;
    else
      return tid-total_length;
  }

  int get_right_succ(int tid) {
    stapl_assert( ((tid>=0)&&(tid<num_elems)), "received invalid task id!" );
    if ( ((tid+1)%total_length) == 0 )
      return -1;
    else
      return tid+1;
  }

  int get_bottom_succ(int tid) {
    stapl_assert( ((tid>=0)&&(tid<num_elems)), "received invalid task id!" );
    if ( (tid/(total_length*(breadth-1))) == 0 )
      return tid + total_length;
    else
      return -1;
  }

  int get_num_preds(int tid) {
    stapl_assert( ((tid>=0)&&(tid<num_elems)), "received invalid task id!" );
    int pred_left = get_left_pred(tid);
    int pred_top  = get_top_pred(tid);
    if ( (pred_left==-1) && (pred_top==-1) )
      return 0;
    else if ( (pred_left!=-1) && (pred_top!=-1) )
      return 2;
    else
      return 1;
  }

  int get_num_succs(int tid) {
    stapl_assert( ((tid>=0)&&(tid<num_elems)), "received invalid task id!" );
    int succ_right  = get_right_succ(tid);
    int succ_bottom = get_bottom_succ(tid);
    if ( (succ_right==-1) && (succ_bottom==-1) )
      return 0;
    else if ( (succ_right!=-1) && (succ_bottom!=-1) )
      return 2;
    else
      return 1;
  }

};

// these flags are used to mark whether a paragraph has started execution or not
bool g_paragraph_flags[2] = { false, false };
// this flag is used to track if overlapped execution has been observed or not
bool overlapped_execution = false;

struct empty_wf
{
  // data member to keep track of which paragraph is executing
  int          m_paragraph_id;

  typedef void result_type;

  empty_wf(int paragraph_id)
    : m_paragraph_id(paragraph_id)
  { }

  void define_type(stapl::typer& t)
  {
    t.member(m_paragraph_id);
  }

  result_type operator()() const
  {
    // mark this paragraph to be executing
    g_paragraph_flags[m_paragraph_id] = true;

    // if I am the first paragraph and the second paragraph has started
    // execution then overlapped execution is taking place
    if (m_paragraph_id==0 && g_paragraph_flags[1])
      overlapped_execution = true;
  }
};

struct viewless_factory
  : public stapl::task_factory_base
{
private:
  id_calculator id_calc;
  int           m_paragraph_id;
  bool          m_invert;

public:
  using result_type = empty_wf::result_type;
  using internal_result_type = empty_wf::result_type;

  viewless_factory(size_t length, size_t breadth, int paragraph_id,
                   bool invert=false)
    : id_calc(length, breadth, stapl::get_num_locations()),
      m_paragraph_id(paragraph_id),
      m_invert(invert) { }

  template<typename TGV>
  void operator()(TGV const& tgv)
  {
    // get the tasks to be added from the id_calculator
    int id = (int)stapl::get_location_id();
    std::vector<int> vec_tasks = id_calc.get_task_ids(id);
    stapl_assert(!vec_tasks.empty(), "vector of tasks to be added is empty");

    // add the tasks
    std::vector<int>::iterator  vec_it = vec_tasks.begin();
    for (; vec_it != vec_tasks.end(); ++vec_it) {
      int tid       = *vec_it;
      int num_preds = 0; int num_succs = 0;
      int pred_left = 0; int pred_top  = 0;
      if (m_invert) {
        num_preds = id_calc.get_num_succs(tid);
        num_succs = id_calc.get_num_preds(tid);
        pred_left = id_calc.get_right_succ(tid);
        pred_top  = id_calc.get_bottom_succ(tid);
      }
      else {
        num_preds = id_calc.get_num_preds(tid);
        num_succs = id_calc.get_num_succs(tid);
        pred_left = id_calc.get_left_pred(tid);
        pred_top  = id_calc.get_top_pred(tid);
        //int succ_right  = id_calc.get_right_succ(tid);
        //int succ_bottom = id_calc.get_bottom_succ(tid);
      }

      switch(num_preds) {
        case 0: {
          tgv.add_task((size_t)tid,empty_wf(m_paragraph_id),(size_t)num_succs);
        } break;

        case 1: {
          tgv.add_task( (size_t)tid, empty_wf(m_paragraph_id),
                    (pred_left!=-1) ?
                      std::vector<std::size_t>(1, (size_t)pred_left)
                      : std::vector<std::size_t>(1, (size_t)pred_top),
                    (size_t)num_succs);
        } break;

        case 2: {
          std::vector<std::size_t> pred_vec;
          pred_vec.push_back((size_t)pred_left);
          pred_vec.push_back((size_t)pred_top);
          tgv.add_task( (size_t)tid, empty_wf(m_paragraph_id),
                        pred_vec, (size_t)num_succs);
        } break;

        default: {
          stapl_assert(false, "Unknown number of predecessors");
        } break;
      }
    }

    //stapl::rmi_fence();
    this->m_finished = true;
  }

  void reset() { this->m_finished = false; }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using std::cout;
  using std::endl;
  using namespace stapl;

  size_t num_locations = get_num_locations();
  size_t pid           = get_location_id();

  if (pid == 0)
    cout << "Testing paragraph for overlapped execution on "
         << num_locations << " locations...";

  typedef counter<default_timer> counter_type;
  counter_type c;

  stapl::rmi_fence();

  int length           = 4* num_locations;
  //int total_length     = length*num_locations;
  int breadth          = 4;

  typedef paragraph<default_scheduler, viewless_factory> tg_t;
  typedef array<bool>                                    cnt_t;
  typedef array_view<cnt_t>                              vw_t;

  cnt_t                                               ct(num_locations);
  vw_t                                                vw(ct);

#ifndef STAPL_NDEBUG
  bool result = false;
#endif

  rmi_fence();
  c.start();

  // test blocking paragraphs
  // clear the flags
  g_paragraph_flags[0]    = false;
  g_paragraph_flags[1]    = false;
  overlapped_execution = false;

  // execute blocking paragraphs
  tg_t(viewless_factory(length, breadth, 0))();
  tg_t(viewless_factory(length, breadth, 1, true))();
  vw[pid] = overlapped_execution;
#ifndef STAPL_NDEBUG
  result  = map_reduce(identity<bool>(), logical_or<bool>(), vw);
  stapl_assert(!result,
               "overlapped execution detected when it is not possible");
#else
  map_reduce(identity<bool>(), logical_or<bool>(), vw);
#endif
  c.stop();

  //cout  << "pid: " << pid <<":: "
  //      << total_length << " x " << breadth
  //      << " grid" << ", time: " << c.value()
  //      << endl;

  rmi_fence();
  c.reset();
  c.start();

  // test non blocking paragraphs
  // clear the flags
  g_paragraph_flags[0] = false;
  g_paragraph_flags[1] = false;
  overlapped_execution = false;

  // execute non blocking paragraphs
  (*new tg_t(viewless_factory(length, breadth, 0)))((int) 0);
  (*new tg_t(viewless_factory(length, breadth, 1, true)))((int) 0);

  vw[pid] = overlapped_execution;
#ifndef STAPL_NDEBUG
  result  = map_reduce(identity<bool>(), logical_or<bool>(), vw);

  /// FIXME - this test needs to be updated to specify a round robin scheduler
  /// (which is really what it is testing...)
  ///
  /// if (num_locations > 1)
  ///   stapl_assert(result,
  ///     "overlapped execution not detected when it should have been");
#else
  map_reduce(identity<bool>(), logical_or<bool>(), vw);
#endif

  c.stop();

  // cout  << "pid: " << pid <<":: "
  //       << total_length << " x " << breadth
  //       << " grid" << ", time: " << c.value()
  //       << endl;

  // if we are here, then it means that the test passed.
  //
  if (pid == 0)
    cout << "Passed" << endl;

  return EXIT_SUCCESS;
}

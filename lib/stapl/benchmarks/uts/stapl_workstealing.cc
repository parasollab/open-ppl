/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <iostream>
#include <stapl/runtime.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/skeletons/map_sched.hpp>
#include <stapl/skeletons/utility/dynamic_wf.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/runtime/executor/scheduler/work_stealing_scheduler.hpp>
#include <stapl/views/counting_view.hpp>

#include "uts.h"
#include "uts_stapl.h"


// some hooks required by uts_stapl.cc (derived from uts_dm.c)
// for STAPL it does not make sense to have uts_stapl.cc and
// stapl_workstealing.cc as separate files. Eventually they
// should be combined. This will eliminate all this hook
// business.

void ss_abort(int error)
{
  stapl::abort(error);
}

void ss_error(char *str)
{
  printf("*** [Location %i] %s\n", ss_get_thread_num(), str);
  stapl::abort();
}

int ss_get_thread_num()
{
  return stapl::get_location_id();
}

int ss_get_num_threads()
{
  return stapl::get_num_locations();
}

char const* ss_get_par_description()
{
  return "STAPL Workstealing";
}


// nodes_per_task
static int nodes_per_task = 20;


// init workfunction
struct spawn_wf
  : public stapl::dynamic_wf
{
  typedef void               result_type;

  // stored as a pointer (and not reference) since
  // ARMI cannot pack references to p_objects.
  StealStack* const m_stealStack_ptr;
  std::vector<Node> m_nodes;

  spawn_wf(StealStack* const ss_ptr, std::vector<Node> const& nodes)
    : m_stealStack_ptr(ss_ptr), m_nodes(nodes)
  { }

  template<typename TGV>
  void operator()(TGV const& tgv)
  {
    // generate children and spawn tasks to process them
    nodes_list_t children(nodes_per_task);

    typename std::vector<Node>::iterator it = m_nodes.begin();

    for (; it!=m_nodes.end(); ++it)
      genChildren(*it, children, *m_stealStack_ptr);


    // get the list
    std::list< std::vector<Node> >& tmp_nodes = children.m_nodes;

    std::list< std::vector<Node> >::iterator tmp_it = tmp_nodes.begin();

    while (tmp_it!=tmp_nodes.end())
    {
      /* count the number of tree nodes */
      m_stealStack_ptr->nNodes += (*tmp_it).size();

      typedef stapl::work_stealing_scheduler<>::sched_info_type si_type;
      tgv.add_task(si_type(),
                   spawn_wf(m_stealStack_ptr, *tmp_it),
                   localize_ref(tgv));

      ++tmp_it;
    }
  }

  void define_type(stapl::typer& t)
  {
    t.base<stapl::dynamic_wf>(*this);
    t.member(m_stealStack_ptr);
    t.member(m_nodes);
  }
};


// spawn workfunction
struct init_wf
  : public stapl::dynamic_wf
{
  StealStack* const m_stealStack_ptr;
  size_t            m_loc_id;

  typedef void result_type;

  init_wf(StealStack* const ss_ptr, size_t loc_id)
    : m_stealStack_ptr(ss_ptr), m_loc_id(loc_id)
  { }

  template<typename TGV, typename Index>
  void operator()(TGV const& tgv, Index const&) const
  {
    if (m_loc_id == 0)
    {
      // add the root task
      Node root;
      uts_initRoot(&root, type);

      /* count the number of tree nodes */
      m_stealStack_ptr->nNodes++;

      typedef stapl::work_stealing_scheduler<>::sched_info_type si_type;

      tgv.add_task(si_type(),
                   spawn_wf(m_stealStack_ptr, std::vector<Node>(1, root)),
                   localize_ref(tgv));
    }
  }

  void define_type(stapl::typer& t)
  {
    stapl::abort("init_wf unexpectedly serialized");
    t.member(m_stealStack_ptr);
    t.member(m_loc_id);
  }
};


template<typename StealPolicy>
void exec_uts_with_policy(std::string policy_name,
                          problem_params_t const& problem_params)
{
  using namespace stapl;

  stapl::do_once([&] { std::cout << "[" << policy_name << "]: "; });

  // create the steal stack
  StealStack ss;

  const int nelems = stapl::get_num_locations();

  /* Initalize trace collection structures */
  ss_initStats(&ss);

  /* show parameter settings */
#if 0
  stapl::do_once([] { uts_printParams() });
#endif

  /* time parallel search */
  double t1 = uts_wctime();

  map_func_sched(
    stapl::work_stealing_scheduler<StealPolicy>(problem_params.m_chunk_size,
                                                problem_params.m_fraction,
                                                problem_params.m_poll_min,
                                                problem_params.m_poll_max,
                                                problem_params.m_poll_step
                                               ),
    init_wf(&ss, stapl::get_location_id()),
    counting_view<int>(nelems, 1));

  double t2 = uts_wctime();

  ss.walltime = t2 - t1;

  // verify the stats and also display them
  showStats(ss, false);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  problem_params_t problem_params(1, 2, 1, 1024, 2, 20);

  /* determine benchmark parameters */
  uts_parseParams(argc, argv, problem_params);
  nodes_per_task = problem_params.m_nodes_per_task;

  /* execute uts with rk_rk_steal policy */
  exec_uts_with_policy<stapl::rk_rk_steal>("rk_rk_steal", problem_params);

  /* execute uts with rk_steal policy */
  exec_uts_with_policy<stapl::rk_steal>("rk_steal", problem_params);

  /* execute uts with neighbor_random_hybrid_steal policy */
  exec_uts_with_policy<stapl::neighbor_random_hybrid_steal>(
    "neighbor_random_hybrid_steal", problem_params);

  /* execute uts with steal_from_previous policy */
  exec_uts_with_policy<stapl::steal_from_previous>("steal_from_previous",
                                                   problem_params);

  /* execute uts with random_lifeline_steal policy */
  exec_uts_with_policy<stapl::random_lifeline_steal>("random_lifeline_steal",
                                                     problem_params);

  /* execute uts with circular_steal policy */
  exec_uts_with_policy<stapl::circular_steal>("circular_steal", problem_params);

  /* execute uts with circular_lifeline_steal policy */
  exec_uts_with_policy<stapl::circular_lifeline_steal>(
    "circular_lifeline_steal", problem_params);

  /* execute uts with diffusive_steal policy */
  exec_uts_with_policy<stapl::diffusive_steal>("diffusive_steal",
                                               problem_params);

  return EXIT_SUCCESS;
}

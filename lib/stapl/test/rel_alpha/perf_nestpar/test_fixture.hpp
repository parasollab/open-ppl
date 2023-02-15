/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef NESTPAR_TEST_FIXTURE
#define NESTPAR_TEST_FIXTURE

#include "work_functors.hpp"
#include "nestpar_utilities.hpp"

#include <stapl/array.hpp>
#include <sstream>

typedef stapl::array<size_t> ary_sz_tp;
typedef stapl::array_view<ary_sz_tp> ary_sz_vw_tp;

template<typename Container, typename ContainerView>
class performance_test_wrapper
{
private:
  Container& m_cont;
  ContainerView& m_vw;

  stapl::counter<stapl::default_timer> ctr;
  std::vector<double> times;
  stapl::array<std::vector<double> > merged_times;
  std::vector<double> longest_times;

public:

  performance_test_wrapper(Container & container,ContainerView & view)
    : m_cont(container), m_vw(view), merged_times(stapl::get_num_locations())
  {  set_random_seed(); }

  template<typename InnerFillWF>
  void fill(InnerFillWF&& inner_fill_wf)
  {
    ctr.reset();
    ctr.start();
    inner_fill_wf(m_vw);
    ctr.stop();
    times.push_back(ctr.value());

  }

  template<typename ProcessWF>
  void run_test(ProcessWF& proc_wf)
  {

    ContainerView m_vw_2(m_cont);

    ctr.reset();
    ctr.start();

    proc_wf(m_vw_2);

    ctr.stop();
    times.push_back(ctr.value());
  }

  std::vector<double> merge_times()
  {
    typedef stapl::identity<std::vector<double> > id_un_wf;
    typedef stapl::max<size_t> max_size_t_wf;
    merged_times[stapl::get_location_id()] = times;
    stapl::rmi_fence();
    typedef stapl::array_view<
              stapl::array<
                std::vector<double> > > times_array_vw;
    times_array_vw a_vw(merged_times);
    return stapl::map_reduce( id_un_wf(), nestpar_perf::max_vec_wf(), a_vw);
  }

  // Output :
  // Step: Fill
  // Time: 0.00360455
  // Step: Run
  // Time: 3.3772e-05
  void save_results()
  {

    longest_times = merge_times();

    std::stringstream ss;
    ss << "Status: PASS" << std::endl;
    //Fill
    ss << "Step: Fill" << std::endl;
    ss << "Time: " << longest_times[0] << std::endl;
    //Run
    ss << "Step: Run" << std::endl;
    ss << "Time: " << longest_times[1] << std::endl;

    test_report(ss.str());
    stapl::rmi_fence();

  }



};

#endif

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef PERF_NESTPAR_TEST6
#define PERF_NESTPAR_TEST6
#include "nestpar_utilities.hpp"
#include "test_fixture.hpp"

#include <stapl/array.hpp>

//Simple map_func of Map_func w/ counting view to fill
size_t perf_test_06( size_t model )
{
  size_t size = 100 * model;

  vec_set_int cont(size);
  vec_set_int_vw view(cont);

  stapl::rmi_fence();
  performance_test_wrapper<
    vec_set_int,
    vec_set_int_vw> test_case(cont,view);

  nestpar_outer_fill_wf<nestpar_inner_set_fill_wf> two_lvl_fill(size);
  test_case.fill(two_lvl_fill);

  nestpar_map_reduce_2lvl_wf proc;
  test_case.run_test(proc);

  test_case.save_results();

  stapl::rmi_fence();
  return 0;
}

#endif

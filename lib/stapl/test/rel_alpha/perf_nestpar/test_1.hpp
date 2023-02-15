/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef PERF_NESTPAR_TEST1
#define PERF_NESTPAR_TEST1
#include "nestpar_utilities.hpp"
#include "test_fixture.hpp"

#include <stapl/array.hpp>

size_t perf_test_01( size_t model )
{
  size_t size = 100 * model;

  typedef stapl::indexed_domain<long long int>            domain_type;
  domain_type dom(1, std::numeric_limits<long long int>::max() - 1);

  set_int cont(dom);
  set_int_vw view(cont);

  performance_test_wrapper<
    set_int,
    set_int_vw> test_case(cont,view);

  nestpar_inner_set_fill_wf inner_fill(size);
  test_case.fill(inner_fill);

  nestpar_map_reduce_process_wf proc;
  test_case.run_test(proc);

  test_case.save_results();

  stapl::rmi_fence();
  return 0;
}

#endif

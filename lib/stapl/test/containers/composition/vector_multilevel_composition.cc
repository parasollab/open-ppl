/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cmath>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "multilevel_composition.hpp"
#include "../../test_report.hpp"


using namespace stapl;

#ifndef NUM_COMP_LEVELS
#define NUM_COMP_LEVELS 3
#endif

template<int N>
struct init_values
{
  typedef void result_type;

  template <typename View>
  void operator()(View v) const
  {
    stapl::map_func(init_values<N-1>(), v);
  }
};


template<>
struct init_values<2>
{
  typedef void result_type;

  template <typename View>
  void operator()(View v) const
  {
    copy(counting_view<int>(v.size(), 1), v);
  }
};


template<typename U, int N>
struct sum_values
{
  typedef U result_type;

  template <typename View>
  U operator()(View v) const
  {
    return stapl::map_reduce(sum_values<U,N-1>(), stapl::plus<U>(), v);
  }
};


template<typename U>
struct sum_values<U,2>
{
  typedef U result_type;

  template <typename View>
  U operator()(View v) const
  {
    return accumulate(v, 0);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  const size_t n = atoi(argv[1]);

  typedef COMPOSE_CONTAINERS_type(NUM_COMP_LEVELS,vector,int) container_t;
  container_t COMPOSE_CONTAINERS_cntr(NUM_COMP_LEVELS,c,n);

  array_view<container_t> v(c);

  stapl::map_func(init_values<NUM_COMP_LEVELS>(),v);

  int res = stapl::map_reduce(
    sum_values<int,NUM_COMP_LEVELS>(), stapl::plus<int>(), v
  );

  int val = (n*(n+1)/2)*std::pow(n,NUM_COMP_LEVELS-1);

  std::string mtype = TEST_msg(NUM_COMP_LEVELS,vector,int);
  std::string msg = "Testing " + mtype;
  STAPL_TEST_REPORT(res == val, msg);

  return EXIT_SUCCESS;
}

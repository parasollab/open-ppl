/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cmath>

#include <stapl/containers/array/array.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/proxy.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include <stapl/algorithms/algorithm.hpp>

#include "multilevel_composition.hpp"
#include "../../test_report.hpp"


using namespace stapl;

#ifndef NUM_COMP_LEVELS
#define NUM_COMP_LEVELS 3
#endif

struct copy_wf
{
  typedef void result_type;

  template <typename T1, typename T2>
  void operator()(T1 t1, T2 t2)
  {
    t2 = t1;
  }
};

template<int N>
struct init_values
{
  typedef void result_type;

  template <typename T>
  void operator()(T t)
  {
    stapl::map_func(init_values<N-1>(), t.property());
  }
};

template<>
struct init_values<2>
{
  typedef void result_type;

  template <typename T>
  void operator()(T t)
  {
    stapl::map_func(copy_wf(),
                    counting_view<int>(
                      t.property().size(),1),
                    t.property());
  }
};

struct extract_wf
{
  typedef int result_type;
  template <typename T>
  result_type operator()(T t)
  {
    return t;
  }
};

template<typename U, int N>
struct sum_values
{
  typedef U result_type;

  template <typename T>
  U operator()(T t)
  {
    return stapl::map_reduce(sum_values<U,N-1>(), stapl::plus<U>(),
                             t.property());
  }
};

template<typename U>
struct sum_values<U,2>
{
  typedef U result_type;

  template <typename T>
  U operator()(T t)
  {
    return stapl::map_reduce(extract_wf(), stapl::plus<int>(),
                             t.property());
  }
};


void test_graph(size_t n)
{
  // compose graph<graph<array<int>>>:

  typedef array<int>  container_0t;
  typedef graph<DIRECTED, MULTIEDGES, container_0t, int> container_1t;
  typedef graph< DIRECTED, MULTIEDGES, container_1t , int > container_t;

  container_t COMPOSE_CONTAINERS_cntr(NUM_COMP_LEVELS,c,n);

  graph_view<container_t> v(c);

  stapl::map_func(init_values<NUM_COMP_LEVELS>(),v);

  int res = stapl::map_reduce(sum_values<int,NUM_COMP_LEVELS>(),
                              stapl::plus<int>(), v);

  int val = (n*(n+1)/2)*std::pow(n,NUM_COMP_LEVELS-1);

  std::string mtype = "graph< graph< array< int > > >";
  std::string msg = "Testing " + mtype;
  STAPL_TEST_REPORT(res == val, msg);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  const size_t n = atoi(argv[1]);

  test_graph(n);

  return EXIT_SUCCESS;
}

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/unordered_set/unordered_set.hpp>
#include <stapl/containers/unordered_multiset/multi_key.hpp>
#include <stapl/views/set_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/runtime/counter/default_counters.hpp>

#include "../../test_report.hpp"

using stapl::default_timer;
using stapl::counter;

struct setwf
{
  typedef size_t result_type;

  template <typename T, typename U>
  result_type operator()(T const& x, U const& y)
  {
    return (x + y);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  const size_t n = atoi(argv[1]);

  typedef stapl::unordered_set<size_t> container_type;
  typedef stapl::set_view<container_type> view_type;


  container_type m;
  container_type n1;
  container_type o;
  STAPL_TEST_REPORT(true, "Testing unordered_set default constructor");

  size_t block = n / stapl::get_num_locations();
  size_t start = block * stapl::get_location_id();
  size_t end = start + block;

  for (size_t i = start; i < end; ++i) {
    m.insert(i);
    n1.insert(i);
    o.insert(i);
  }
  stapl::rmi_fence();
  STAPL_TEST_REPORT((m.size() == n), "Testing insert");

  view_type v(m);
  view_type v1(n1);
  view_type v2(o);

  size_t x = stapl::map_reduce(setwf(), stapl::plus<size_t>(), v, v1);
  STAPL_TEST_REPORT((x == n*(n-1)),
                    "Testing map_reduce over stapl::unordered_set");

  return EXIT_SUCCESS;
}

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/unordered_map/unordered_map.hpp>
#include <stapl/views/map_view.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../../test_report.hpp"

struct mapwf
{
  typedef size_t result_type;

  template <typename U>
  result_type operator()(U const& x)
  {
    return x.second;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  const size_t n = atoi(argv[1]);

  typedef stapl::unordered_map<size_t, size_t> container_type;
  typedef stapl::map_view<container_type> view_type;

  container_type m;
  STAPL_TEST_REPORT(true, "Testing unordered_map default constructor");

  size_t nlocs = stapl::get_num_locations();
  size_t this_loc = stapl::get_location_id();

  for (size_t i = this_loc; i < n; i+=nlocs)
    m.insert(i, i*2);
  stapl::rmi_fence();
  STAPL_TEST_REPORT((m.size() == n), "Testing insert");

  size_t y = m[n/2];
  size_t x = m.apply_get(n/2,
                       stapl::identity<container_type::value_type>()).second;
  stapl::rmi_fence();
  STAPL_TEST_REPORT(x==y && x==((n/2)*2), "Testing apply_get and operator[]");

  view_type v(m);
  size_t xx = stapl::map_reduce(mapwf(), stapl::plus<size_t>(), v);
  STAPL_TEST_REPORT((xx == n*(n-1)),
                    "Testing map_reduce over stapl::unordered_map");

  return EXIT_SUCCESS;
}

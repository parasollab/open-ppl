/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/unordered_multimap/unordered_multimap.hpp>
#include <stapl/views/map_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include "../../test_report.hpp"

using namespace stapl;

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
  size_t locs = get_num_locations();
  size_t this_loc = get_location_id();

  typedef stapl::unordered_multimap<size_t, size_t> container_type;
  typedef stapl::map_view<container_type> view_type;

  container_type m;
  stapl::rmi_fence();
  STAPL_TEST_REPORT(m.size() == 0, "Testing with default constructor   ");

  for (size_t i = this_loc; i < n; i += locs)
  {
    m.insert(i, i);
    m.insert(i, i);
  }
  stapl::rmi_fence();
  STAPL_TEST_REPORT(m.size() == n*2, "Testing insert");

  view_type v(m);
  size_t sum = stapl::map_reduce(mapwf(), stapl::plus<size_t>(), v);
  STAPL_TEST_REPORT(sum == n*(n-1), "Testing map_reduce");

  return EXIT_SUCCESS;
}

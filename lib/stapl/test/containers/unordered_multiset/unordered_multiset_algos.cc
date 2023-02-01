/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/unordered_multiset/unordered_multiset.hpp>
#include <stapl/views/set_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include "../../test_report.hpp"

using namespace stapl;

struct multisetwf
{
  typedef size_t result_type;

  template <typename U>
  result_type operator()(U const& x)
  {
    return x;
  }
};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  const size_t n = atoi(argv[1]);
  size_t locs = get_num_locations();

  typedef stapl::unordered_multiset<size_t>    container_type;
  typedef stapl::set_view<container_type> view_type;

  container_type s;
  STAPL_TEST_REPORT(true, "Testing with default constructor------------------");

  for (size_t i = 0; i != n; ++i)
  {
    s.insert(i);
    s.insert(i);
  }
  stapl::rmi_fence();
  STAPL_TEST_REPORT((s.size() == 2*n*locs), "Testing insert");

  view_type v(s);
  size_t sum = stapl::map_reduce(multisetwf(), stapl::plus<size_t>(), v);
  STAPL_TEST_REPORT(sum == locs*n*(n-1), "Testing map_reduce");

  return EXIT_SUCCESS;
}

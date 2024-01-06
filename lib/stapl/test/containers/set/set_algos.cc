/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/algorithms/algorithm.hpp>

#include <stapl/views/set_view.hpp>
#include <stapl/containers/set/set.hpp>

#include "../../test_report.hpp"

using namespace stapl;

struct mapwf
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

  typedef stapl::set<size_t>               container_type;
  typedef indexed_domain<size_t>           domain_type;
  typedef stapl::set_view<container_type>  view_type;

  domain_type dom(0, n-1);

  container_type s(dom);
  STAPL_TEST_REPORT(true, "Testing set(domain) constructor");

  size_t block = n / get_num_locations();
  size_t start = block * get_location_id();
  size_t end   = start + block;

  for (size_t i = start; i < end; ++i)
    s.insert(i);
  rmi_fence();
  STAPL_TEST_REPORT((s.size() == n), "Testing insert");

  size_t y = s[n/2];
  size_t x = s.apply_get(n/2,
                       stapl::identity<container_type::value_type>());
  stapl::rmi_fence();
  STAPL_TEST_REPORT((x==y ),
                    "Testing apply_get and operator[]");

  view_type v(s);
  size_t xx = stapl::map_reduce(mapwf(), stapl::plus<size_t>(), v);
  STAPL_TEST_REPORT((xx == n*(n-1)/2), "Testing map_reduce over stapl::set");

  return EXIT_SUCCESS;
}

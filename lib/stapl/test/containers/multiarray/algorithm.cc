/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../../test_report.hpp"

#include <stapl/views/metadata/container/multiarray.hpp>

using namespace stapl;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  using stapl::fill;
  using stapl::all_of;
  using stapl::partial_sum;

  if (argc < 4)
  {
    std::cout<< "usage: exe n m r" <<std::endl;
    exit(1);
  }

  size_t n = atol(argv[1]);
  size_t m = atol(argv[2]);
  size_t r = atol(argv[3]);

  typedef size_t                           value_type;
  typedef multiarray<3, value_type>        multiarray_type;
  typedef multiarray_view<multiarray_type> view_type;
  typedef multiarray_type::traversal_type  traversal_type;
  typedef multiarray_type::dimensions_type dimensions_type;

  dimensions_type s = dimensions_type(n, m, r);
  multiarray_type c(s);

  view_type v(c);
  auto linear = linear_view(v);

  fill(v, n);

  STAPL_TEST_REPORT(true, "Testing map_func over linear multiarray view");

  bool b = all_of(linear, boost::bind(stapl::equal_to<value_type>(), _1, n));

  STAPL_TEST_REPORT(b, "Testing map_reduce over linear multiarray view");

  partial_sum(linear, linear);

  b = !all_of(linear, boost::bind(stapl::equal_to<value_type>(), _1, n));

  STAPL_TEST_REPORT(b, "Testing prefix_scan over linear multiarray view");

  return EXIT_SUCCESS;
}

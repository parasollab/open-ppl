/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/counting_view.hpp>

#include "../../test_report.hpp"

using namespace stapl;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: n" << std::endl;
    exit(1);
  }

  size_t n = atoi(argv[1]);

  typedef vector<size_t>            vector_type;
  typedef vector_view<vector_type>  view_type;

  size_t result = (n*(n-1))/2;

  vector_type a(n);
  view_type v(a);

  ::stapl::map_func(stapl::assign<size_t>(), counting_view<size_t>(n), v);
  size_t res = ::stapl::map_reduce(stapl::identity<size_t>(), stapl::plus<size_t>(), v);

  STAPL_TEST_REPORT((res == result) ,"Testing algorithms map/reduce on vector");

  return EXIT_SUCCESS;
}

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/views/strided_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/numeric.hpp>

#include <cmath>

#include "../test_report.hpp"

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: exe n" << std::endl;
    exit(1);
  }

  const size_t n = atoi(argv[1]);

  using std::floor;
  const size_t exp_sums[] = {
    size_t( n*(-1 + 2*n) ),
    size_t( (-1 + n)*n ),
    size_t( (3*floor((-1 + 2*n)/3.)*(1 + floor((-1 + 2*n)/3.)))/2. ),
    size_t( n*(-1 + 2*n) ),
    size_t( n*n ),
    size_t( ((1 + floor((2*(-1 + n))/3.))*(2 + 3*floor((2*(-1 + n))/3.)))/2. ),
    size_t( (-1 + n)*(1 + 2*n) ),
    size_t( (-1 + n)*n ),
    size_t( (floor((2*n)/3.)*(1 + 3*floor((2*n)/3.)))/2. ),
    size_t( (1 + n)*(-3 + 2*n) ),
    size_t( -1 + n*n ),
    size_t( (3*(1 + floor((2*(-2 + n))/3.))*(2 + floor((2*(-2 + n))/3.)))/2. )
  };

  auto counting = stapl::counting_view<size_t>(2*n);

  size_t const* exp_sum = &exp_sums[0];

  for (int start = 0; start < 4; ++start)
    for (int step = 1; step < 4; ++step)
    {
      auto sv = stapl::make_strided_view(counting, step, start);

      std::stringstream ss;
      ss << "Testing strided_view, start=" << start << ", step=" << step;
      STAPL_TEST_REPORT(stapl::accumulate(sv, 0ul) == *exp_sum++, ss.str())
    }

  STAPL_TEST_REPORT(
    stapl::accumulate(stapl::make_strided_view(counting, 2*n-1, 0), 0ul)==2*n-1,
    "Testing strided_view, start=0, step=view.size()-1")

  STAPL_TEST_REPORT(
    stapl::accumulate(stapl::make_strided_view(counting, 2*n-1, 1), 0ul) == 1,
    "Testing strided_view, start=1, step=view.size()-1")

  return EXIT_SUCCESS;
}

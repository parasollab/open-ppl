/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <stapl/runtime.hpp>
#include <stapl/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/skeletons/executors/hand_execute.hpp>
#include <stapl/skeletons/functional/map_reduce.hpp>

#include "../test_report.hpp"

struct zero_if_even
{
  template<typename T>
  int operator()(T x) const
  {
    return x % 2 == 0 ? 0 : x;
  }
};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << argv[0] << " <nElem>" << std::endl;
    return EXIT_FAILURE;
  }

  const std::size_t n = atol(argv[1]);

  stapl::array<int> a{n};
  auto vw = make_array_view(a);
  stapl::iota(vw, 1);

  auto skel = stapl::skeletons::map_reduce(zero_if_even{}, stapl::plus<int>{});

  // The sum of the first n odd numbers is n^2
  int sum = stapl::skeletons::hand_execute(skel, vw);
  int number_of_odds = std::ceil(static_cast<double>(n)/2);

  STAPL_TEST_REPORT(sum == number_of_odds * number_of_odds,
                    "hand_execute map_reduce");

  return EXIT_SUCCESS;
}

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/array.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/views/metadata/base_container_range.hpp>

#define REPORT_WITH_COLOR
#include "../../test_report.hpp"

stapl::exit_code stapl_main(int argc, char* argv[])
{
  constexpr std::size_t n = 1024;

  stapl::array<int> c{n};
  stapl::array_view<stapl::array<int>> v{c};

  auto rng = stapl::metadata::make_base_container_range(v);

  for (auto& bc : rng)
    for (auto x : bc)
      x = 1;

  const std::size_t sum = stapl::accumulate(v, 0);
  STAPL_TEST_ALL_LOCATION_REPORT(sum == n, "Access to raw elements");

  auto b = rng.begin();
  auto e = rng.end();

  STAPL_TEST_ALL_LOCATION_REPORT(std::distance(b, e) == 1,
                                 "One base container per location");

  const auto loc = stapl::get_location_id();
  STAPL_TEST_ALL_LOCATION_REPORT(b->size()
                                   == c.distribution().partition()[loc].size(),
                                 "First base container in range is valid");

  return EXIT_SUCCESS;
}

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
#include <stapl/views/metadata/extract.hpp>

#include "../../../test_report.hpp"

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: exe n" << std::endl;
    exit(1);
  }

  const std::size_t n = atoi(argv[1]);

  using array_type = stapl::array<int>;
  using view_type = stapl::array_view<array_type>;

  array_type a(n);
  view_type v(a);

  auto md_cont = stapl::metadata::extract(v);

  bool passed = md_cont->size() == stapl::get_num_locations();
  STAPL_TEST_REPORT(passed, "Size is correct");

  return EXIT_SUCCESS;
}

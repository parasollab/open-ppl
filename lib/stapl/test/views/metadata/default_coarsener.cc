/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../../test_report.hpp"
#include "utils.hpp"

using namespace stapl;


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe n" << std::endl;
    exit(1);
  }

  size_t n = atoi(argv[1]);

  typedef std::string            value_type;
  typedef array<value_type>      array_type;
  typedef array_view<array_type> view_type;

  array_type a(n);
  view_type v(a);

  auto c = coarsen_views(v);
  auto& coarsened = get<0>(c);

  const std::size_t num_chunks = coarsened.size();
  const std::size_t p = get_num_locations();

  STAPL_TEST_REPORT(num_chunks == p,
    "Number of coarsened chunks"
  )

  return EXIT_SUCCESS;
}

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/strided_view.hpp>

#include "../../../test_report.hpp"
#include "../utils.hpp"

using namespace stapl;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cout << "usage: exe n s" << std::endl;
    exit(1);
  }
// FIXME (coarsening): this test needs to be re-enabled when coarsening
// for nd strided view is available
#if 0
  size_t n = atoi(argv[1]);
  size_t s = atoi(argv[2]);

  typedef std::string                      value_type;
  typedef multiarray<3, value_type>        multiarray_type;
  typedef multiarray_view<multiarray_type> view_type;

  multiarray_type a(std::make_tuple(n,n,n));
  view_type v(a);

  auto strided = make_strided_view(
    v, std::make_tuple(s,s,s), std::make_tuple(1,1,1)
  );

  auto c = coarsen_views(strided);
  auto& coarsened = get<0>(c);

  // it was coarsened correctly if the coarsened view covers
  // the space and it is exactly p chunks
  bool passed = coarsening_covers_space(strided, coarsened);
  //passed &= coarsened.size() == get_num_locations();

  STAPL_TEST_REPORT(passed, "Testing multidimensional strided projection");
#endif

  return EXIT_SUCCESS;
}

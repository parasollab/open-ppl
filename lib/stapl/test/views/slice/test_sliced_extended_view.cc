/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/multiarray.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/views/slices_view.hpp>
#include <stapl/views/extended_view.hpp>

#include "../../test_report.hpp"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Workfunction to increment a scalar
//////////////////////////////////////////////////////////////////////
struct increment
{
  typedef void result_type;

  template<typename T>
  void operator()(T x)
  {
    x += 1;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Workfunction that receives a slice of a view, and spawns
///        a nested PARAGRAPH to increment all elements of that slice.
//////////////////////////////////////////////////////////////////////
struct increment_slice
{
  typedef void result_type;

  template<typename View>
  void operator()(View const& slice)
  {
    map_func(increment(), slice);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: exe n" << std::endl;
    exit(1);
  }

  const std::size_t n = atoi(argv[1]);

  using traversal_t     = index_sequence<0, 1, 2>;
  using sliced_dims     = index_sequence<1, 2>;
  using sliced_part_t   =
    typename sliced_md_distribution_spec<sliced_dims, traversal_t>::type;
  using multiarray_type = multiarray<3, int, sliced_part_t>;

  // make multiarray with a 0'th dim of size 1
  multiarray_type sigt(
    sliced_volumetric<traversal_t, sliced_dims>(make_tuple(1,n,n)), 0);

  // make a view over the multiarray
  auto vw = make_multiarray_view(sigt);

  // extend the 0'th dim to size n
  auto sigt_vw = make_extended_view<0>(sigt, n);

  // make a slices view on top of extended view
  auto sigt_sv = make_slices_view<1, 2>(sigt_vw);

  // increment all values in the slices view
  map_func(increment_slice(), sigt_sv);

  bool passed =
    all_of(linear_view(vw), boost::bind(stapl::equal_to<int>(), _1, n));

  STAPL_TEST_REPORT(passed, "Slices view over extended view");

  return EXIT_SUCCESS;
}

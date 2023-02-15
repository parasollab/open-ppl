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
#include <stapl/algorithms/numeric.hpp>
#include <stapl/multiarray.hpp>
#include <stapl/views/slices_view.hpp>
#include <stapl/containers/distribution/specifications.hpp>

#include "../utils.hpp"
#include "../../../test_report.hpp"

using namespace stapl;

template<typename View>
bool test_slices(View const& v)
{
  auto sv = make_slices_view<0, 1, 2>(v);

  auto extracted = extract_metadata_from_view(sv);

  auto result = extracted.second->size() == get_num_locations();

  delete extracted.second;

  return result;
}


template<typename View>
bool test_composed_slices(View const& v)
{
  auto sv1 = make_slices_view<0, 1, 2, 4>(v);
  auto sv2 = make_slices_view<0, 1, 2>(sv1);

  auto extracted = extract_metadata_from_view(sv2);

  auto result = extracted.second->size() == get_num_locations();

  delete extracted.second;

  return result;
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cout << "usage: exe m n" << std::endl;
    exit(1);
  }

  const std::size_t m = atoi(argv[1]);
  const std::size_t n = atoi(argv[2]);

  using traversal_t     = index_sequence<4, 3, 2, 1, 0>;
  using sliced_dims     = index_sequence<0, 1, 2>;
  using sliced_part_t   =
    sliced_md_distribution_spec<sliced_dims, traversal_t>::type;

  using multiarray_type = multiarray<5, int, sliced_part_t>;
  using view_type       = multiarray_view<multiarray_type>;

  auto size = make_tuple(m, m, m, n, n);

  multiarray_type a(sliced_volumetric<traversal_t, sliced_dims>(size));
  view_type v(a);

  bool t0 = test_slices(v);
  STAPL_TEST_REPORT(t0, "Metadata extraction for slices view");

  bool t1 = test_composed_slices(v);
  STAPL_TEST_REPORT(t1, "Metadata extraction for composed slices view");

  return EXIT_SUCCESS;
}

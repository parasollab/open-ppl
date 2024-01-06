/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/array.hpp>
#include <stapl/multiarray.hpp>
#include <stapl/views/slices_view.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/utility/tuple/homogeneous_tuple.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include "../../../test_report.hpp"
#include "test_work_functions.hpp"

using namespace stapl;

template<int D>
using dims_type = homogeneous_tuple_type_t<D, std::size_t>;

void test_1d_1d_1d_2d_balanced(dims_type<5> const& dims)
{
  using multiarray_type = multiarray<5, bool>;

  multiarray_type a(dims);
  auto v = make_multiarray_view(a);

  auto sv1 = make_slices_view<0, 1, 2>(v);
  auto sv2 = make_slices_view<1, 2>(sv1);
  auto sv30 = make_slices_view<0>(sv2);
  auto sv31 = make_slices_view<1>(sv2);

  map_func(lev1(), sv30, sv30);

  auto linear = linear_view(v);
  bool passed = count(linear, true) == v.size();

  fill(linear, false);

  map_func(lev1(), sv31, sv31);
  passed &= count(linear, true) == v.size();

  fill(linear, false);

  map_func(lev1_intermediate_view(), sv31, sv31);
  passed &= count(linear, true) == v.size();

  fill(linear, false);

  auto sv22 = make_multiarray_view(sv2);
  auto sv33 = make_slices_view<1>(sv22);

  map_func(lev1(), sv33, sv33);
  passed &= count(linear, true) == v.size();

  STAPL_TEST_REPORT(passed,
    "Composed slices (balanced distribution) + skeletons (1-1-1-2)");
}

exit_code stapl_main(int argc, char** argv)
{
  const std::size_t m = atoi(argv[1]);
  const std::size_t n = atoi(argv[2]);
  const std::size_t o = atoi(argv[3]);
  const std::size_t p = atoi(argv[4]);
  const std::size_t q = atoi(argv[5]);

  dims_type<5> dims_5d { m, n, o, p, q };

  test_1d_1d_1d_2d_balanced(dims_5d);
  
  return EXIT_SUCCESS;
}

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/views/mapping_functions/linearization.hpp>
#include <stapl/containers/type_traits/make_traversal.hpp>

#include "../../test_report.hpp"
#include "../../confint.hpp"

using namespace stapl;


exit_code stapl_main(int argc, char* argv[])
{
  using tuple_t       = tuple<size_t, size_t, size_t, size_t, size_t>;
  using slice_tuple_t = tuple<size_t, size_t, size_t>;

  tuple_t size_tuple(10, 10, 10, 10, 10);
  tuple_t start_tuple(2, 2, 3, 2, 1);

  //
  // ZDG test
  //
  using zdg_traversal_t = make_traversal<4, 3, 2, 1, 0>::type;
  using zdg_linearize_t = nd_linearize<tuple_t, zdg_traversal_t>;

  zdg_linearize_t zdg_linearize(size_tuple, start_tuple);
  size_t val1 = zdg_linearize(7, 8, 5, 3, 1);

  auto zdg_slice = zdg_linearize.slice<0, 1, 2>(slice_tuple_t(7, 8, 5));
  size_t val2 = zdg_slice.first + zdg_slice.second(3, 1);

  STAPL_TEST_REPORT(val1 == val2,
                    "Testing Linearizer 3D slice, Traversal = (4, 3, 2, 1, 0)");

  //
  // ZGD test
  //
  using zgd_traversal_t = make_traversal<4, 3, 2, 0, 1>::type;
  using zgd_linearize_t = nd_linearize<tuple_t, zgd_traversal_t>;

  zgd_linearize_t zgd_linearize(size_tuple, start_tuple);
  val1 = zgd_linearize(7, 8, 5, 3, 1);

  auto zgd_slice = zgd_linearize.slice<0, 1, 2>(slice_tuple_t(7, 8, 5));
  val2 = zgd_slice.first + zgd_slice.second(3, 1);

  STAPL_TEST_REPORT(val1 == val2,
                    "Testing Linearizer 3D slice, Traversal = (4, 3, 2, 0, 1)");

  //
  // GZD test
  //
  using gzd_traversal_t = make_traversal<3, 2, 1, 0, 4>::type;
  using gzd_linearize_t = nd_linearize<tuple_t, gzd_traversal_t>;

  gzd_linearize_t gzd_linearize(size_tuple, start_tuple);
  val1 = gzd_linearize(7, 8, 5, 3, 1);

  auto gzd_slice_1 = gzd_linearize.slice<4>(1);
  auto gzd_slice_2 = gzd_slice_1.second.slice<0, 1, 2>(slice_tuple_t(7, 8, 5));
  val2 = gzd_slice_1.first + gzd_slice_2.first + gzd_slice_2.second(3);

  STAPL_TEST_REPORT(val1 == val2,
                "Testing Linearizer 1D, 3D slice, Traversal = (3, 2, 1, 0, 4)");
  //
  // DZG
  //
  using dgz_traversal_t = make_traversal<1,2,3,4,0>::type;
  using dgz_linearize_t = nd_linearize<tuple_t, dgz_traversal_t>;

  dgz_linearize_t dgz_linearize(size_tuple, start_tuple);
  val1 = dgz_linearize(7, 8, 5, 3, 1);

  auto dgz_slice_1 = dgz_linearize.slice<3>(3);
  auto dgz_slice_2 = dgz_slice_1.second.slice<std::tuple<
    std::integral_constant<std::size_t, 0>,
    std::integral_constant<std::size_t, 1>,
    std::integral_constant<std::size_t, 2>>>(slice_tuple_t(7, 8, 5));
  val2 = dgz_slice_1.first + dgz_slice_2.first + dgz_slice_2.second(1);

  STAPL_TEST_REPORT(val1 == val2,
                "Testing Linearizer 1D, 3D slice, Traversal = (1, 2, 3, 4, 0)");

  //
  // GDZ test
  //
  using gdz_traversal_t = make_traversal<0, 1, 2, 3, 4>::type;
  using gdz_linearize_t = nd_linearize<tuple_t, gdz_traversal_t>;

  gdz_linearize_t gdz_linearize(size_tuple, start_tuple);
  val1 = gdz_linearize(7, 8, 5, 3, 1);

  auto gdz_slice_1 = gdz_linearize.slice<4>(1);
  auto gdz_slice_2 = gdz_slice_1.second.slice<3>(3);

  val2 = gdz_slice_1.first
         + gdz_slice_2.first
         + gdz_slice_2.second(slice_tuple_t(7, 8, 5));

  STAPL_TEST_REPORT(val1 == val2,
                "Testing Linearizer 1D, 1D slice, Traversal = (2, 1, 0, 3, 4)");

  return EXIT_SUCCESS;
}

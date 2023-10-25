/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/views/extended_view.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include <boost/format.hpp>

#include "../test_report.hpp"

using namespace stapl;


template<int D, int I>
void test_extended(std::size_t n)
{
  using container_type = multiarray<D, std::size_t>;
  using view_type = multiarray_view<container_type>;
  using plus_t = stapl::plus<std::size_t>;

  auto a_size = homogeneous_tuple<D>(n);
  auto b_size = homogeneous_tuple<D>(n);
  get<I>(b_size) = 1;

  container_type a_grid(a_size, 1);
  container_type b_grid(b_size, 0);

  view_type a_view(a_grid);
  view_type b_view(b_grid);

  auto extended_b = make_extended_view<I>(b_grid, n);

  stapl::map_func(
    assign<std::size_t>(), a_view, extended_b
  );

  bool passed = stapl::all_of(linear_view(b_view),
    boost::bind(stapl::equal_to<std::size_t>(), _1, 1)
  );

  const std::string msg =
    str(boost::format("Extended %dd dimension %d") % D % I);

  STAPL_TEST_REPORT(passed, msg);
}


template<int D, int I = 0>
struct test_all_extensions
{
  static void call(std::size_t n)
  {
    test_extended<D, I>(n);

    // recursively test for the next dimension
    test_all_extensions<D, I+1>::call(n);
  }
};


template<int D>
struct test_all_extensions<D, D>
{
  static void call(std::size_t n)
  { }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: exe n" << std::endl;
    exit(1);
  }

  size_t n = atoi(argv[1]);

  test_all_extensions<2>::call(n);
  test_all_extensions<3>::call(n);

  // FIXME (coarsening): these tests are disabled until the wavefront skeleton
  // which is used in multidimensional alignment is generalized to more than 3D.
#if 0
  test_all_extensions<4>::call(n);
  test_all_extensions<5>::call(n);
#endif

  return EXIT_SUCCESS;
}

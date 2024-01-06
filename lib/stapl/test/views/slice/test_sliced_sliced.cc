/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/algorithm.hpp>
#include <stapl/views/sliced_view.hpp>
#include <stapl/utility/integer_sequence.hpp>
#include <stapl/views/counting_view.hpp>
#include "../../test_report.hpp"

using namespace stapl;

template<typename Slice>
std::size_t accumulate_slice(Slice slice)
{
  using index_type = typename Slice::index_type;
  std::size_t sum = 0;

  domain_map(slice.domain(), [&](index_type const& i) {
    tuple<int, int, int, int, int> x = slice[i];
    sum += vs_map_reduce(stapl::identity<int>(), stapl::plus<int>(), 0, x);
  });

  return sum;
}

template<typename CountingView>
void test_sliced_2_2(CountingView v, std::size_t n)
{
  using slices_tup_01 = tuple<
    std::integral_constant<std::size_t, 0>,
    std::integral_constant<std::size_t, 1>
  >;

  auto slice0 = make_SLICED_view<slices_tup_01>(v, 1, 1);
  auto slice1 = slice0.template slice<slices_tup_01>(make_tuple(1, 1));

  STAPL_TEST_REPORT(slice1.size() == n, "Size of outer-most slice (2-2)");

  auto sum = accumulate_slice(slice1);

  STAPL_TEST_REPORT(sum == (n*(n+7))/2, "Accumulate of slice<0,1>(slice<0,1>)");
}

template<typename CountingView>
void test_sliced_2_1(CountingView v, std::size_t n)
{
  using slices_tup_0 = tuple<std::integral_constant<std::size_t, 0>>;
  using slices_tup_01 = tuple<
    std::integral_constant<std::size_t, 0>,
    std::integral_constant<std::size_t, 1>
  >;

  auto slice0 = make_SLICED_view<slices_tup_01>(v, 1, 1);
  auto slice1 = slice0.template slice<slices_tup_0>(1);

  STAPL_TEST_REPORT(slice1.size() == n*n, "Size of outer-most slice (2-1)");

  auto sum = accumulate_slice(slice1);
  std::size_t expected = 2*n*n+n*n*n;

  STAPL_TEST_REPORT(sum == expected, "Accumulate of slice<0>(slice<0,1>)");
}

exit_code stapl_main(int argc, char** argv)
{
  if (argc < 2)
  {
    std::cout << "usage: " << argv[0] << " n" << std::endl;
    exit(1);
  }

  const std::size_t n = atoi(argv[1]);
  const auto size = make_tuple(n, n, n, n, n);

  auto v = counting_view_nd<5>(size, make_tuple(0,0,0,0,0));

  test_sliced_2_2(v, n);
  test_sliced_2_1(v, n);

  return EXIT_SUCCESS;
}

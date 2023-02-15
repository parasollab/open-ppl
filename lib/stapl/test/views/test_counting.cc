/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/skeletons/map_reduce.hpp>
#include <stapl/skeletons/transformations/coarse/zip.hpp>
#include <stapl/skeletons/functional/skeleton_traits.hpp>
#include <stapl/skeletons/spans/blocked.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/containers/multiarray/multiarray.hpp>

#include "../test_report.hpp"

using namespace stapl;


struct compare_wf
{
  template<typename R1, typename R2>
  void operator()(R1&& r1, R2&& r2) const
  {
    tuple<size_t, size_t> tmp = r2;

    if (index_of(r1) != tmp)
    {
      abort("Failure in counting_view_nd test");
      return;
    }
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cout << "usage: exe n" << std::endl;
    exit(1);
  }

  long n = atol(argv[1]);

  long sum = map_reduce(
    stapl::identity<long>(), stapl::plus<long>(), counting_view<long>(n)
  );

  STAPL_TEST_REPORT(
    sum == (n*(n-1))/2,
    "Testing counting_view (default initial value)"
  );

  sum = map_reduce(
    stapl::identity<long>(), stapl::plus<long>(), counting_view<long>(n, -(n/2))
  );

  STAPL_TEST_REPORT(
    ((n%2==0) && (sum==-(n/2)))||(sum==0),
    "Testing counting_view (with initial value)"
  );

  sum = map_reduce(
    stapl::identity<long>(), stapl::plus<long>(), counting_view<long>(2));

  STAPL_TEST_REPORT(
    sum == 1,
    "Testing counting_view (with two elements)"
  );

  //
  // counting_view_nd
  //
  const  size_t n1 = n <= 100 ? n : n / 10;

  auto view =
    counting_view_nd<2>(make_tuple(n1,n1), tuple<size_t, size_t>(0,0));

  using multiarray_t = multiarray<2, size_t>;

  multiarray_t                  ma(make_tuple(n1,n1));
  multiarray_view<multiarray_t> mv(ma);

  using span_t = skeletons::spans::blocked<2>;

  skeletons::execute(
    skeletons::execution_params(multiview_coarsener<true>()),
    skeletons::coarse(
      skeletons::zip<2>(compare_wf(), skeletons::skeleton_traits<span_t>())),
    mv, view);

  STAPL_TEST_REPORT(
    true, "Testing counting_view_nd"
  );

  return EXIT_SUCCESS;
}

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
#include <stapl/skeletons/map_reduce.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/views/transform_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/containers/vector/vector.hpp>

#include "../test_report.hpp"

using namespace stapl;

struct returns_five
{

  using result_type = long;

  template<typename T>
  result_type operator()(T) const
  {
    return 5;
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
    stapl::identity<long>(), stapl::plus<long>(),
    transform_view<decltype(counting_view<long>(n)), returns_five>
      (counting_view<long>(n), returns_five{})
  );

  STAPL_TEST_REPORT(
    sum == 5*n,
    "Testing transform_view (over counting view)"
  );

  stapl::array<int> v(n);
  stapl::transform(stapl::counting_view(n, (long)0),
                   make_array_view(v),
                   identity<long>{});
  auto view = transform_view<decltype(make_array_view(v)), returns_five>
    (v, returns_five{});

  bool all_five = stapl::do_once([&] {
    bool all_five = true;
    for (auto x : view) {
      if (x != 5) {
        all_five = false;
      }
    }
    return all_five;
  });
  stapl::rmi_fence();
  STAPL_TEST_REPORT(
    all_five,
    "Testing transform_view (iterators)"
  );

  all_five = true;
  stapl::do_once([&] {
    for (size_t i=0; i<(size_t)n; i++) {
      if (view[i] != 5) {
        all_five = false;
      }
    }
  });
  stapl::rmi_fence();
  STAPL_TEST_REPORT(
    all_five,
    "Testing transform_view (indexing)"
  );


  stapl::vector<long> vect(n);
  stapl::transform(
      stapl::counting_view(n, (long)0),
      make_vector_view(vect), identity<long>{});
  auto view2 = transform_view<decltype(make_vector_view(vect)),
                              returns_five>(vect, returns_five{});

  stapl::do_once([&] {
    view2.push_back(7);
  });
  stapl::rmi_fence();
  STAPL_TEST_REPORT(
    view2.size() == (size_t)n+1,
    "Testing transform_view (push_back on vector_view increased size)"
  );

  all_five = true;
  stapl::do_once([&] {
    for (auto x : view2) {
      if (x != 5) {
        all_five = false;
      }
    }
  });
  stapl::rmi_fence();
  STAPL_TEST_REPORT(
    all_five,
    "Testing transform_view (push_back on vector_view)"
  );

  return EXIT_SUCCESS;
}

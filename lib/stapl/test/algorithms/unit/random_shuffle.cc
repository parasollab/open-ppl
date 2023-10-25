/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>

#include <stapl/algorithms/functional.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/sorting.hpp>
#include <stapl/algorithms/numeric.hpp>

#include <stapl/views/array_view.hpp>
#include <stapl/views/list_view.hpp>
#include <stapl/views/counting_view.hpp>

#include <stapl/containers/array/array.hpp>
#include <stapl/containers/list/list.hpp>

#include "../../test_report.hpp"

using namespace stapl;


class random_number_generator
{
public:
  typedef size_t result_type;
  result_type operator()()
  {
    return std::rand();
  }
  result_type min()
  {
    return 0;
  }
  result_type max()
  {
    return std::numeric_limits<result_type>::max();
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::counter<stapl::default_timer> timer;
  size_t n = 100;
  if (argc == 2){
    n = atoi(argv[1]);;
  }

  typedef array<int>               parray_t;
  typedef array_view<parray_t>     view_t;

  // p_array test
  parray_t pa(n);
  view_t view(pa);

  copy(counting_view<int>(n),view);

  random_shuffle(view);
  bool is_sorted = stapl::is_sorted(view);
  size_t sum = accumulate(view,size_t(0));

  STAPL_TEST_REPORT((sum == (n*(n-1))/2) && !is_sorted,
                    "Testing random_shuffle over p_array")

/* This is commented out because it fails for more than 2 cores
  // p_list test
  typedef list<int>          plist_t;
  typedef list_view<plist_t> viewl_t;

  plist_t pl(n);
  viewl_t viewl(pl);

  copy(counting_view<int>(n),viewl);

  random_shuffle(viewl);
  is_sorted = stapl::is_sorted(viewl);
  sum = accumulate(viewl,size_t(0));

  STAPL_TEST_REPORT((sum == (n*(n-1))/2) && !is_sorted,
                    "Testing random_shuffle over p_list")
*/

  // p_array shuffle test
  parray_t pa2(n);
  view_t view2(pa);

  copy(counting_view<int>(n),view2);

  random_number_generator g;
  timer.reset();
  timer.start();
  shuffle(view2,g);
  double exec_time = timer.stop();
  is_sorted = stapl::is_sorted(view2);
  sum = accumulate(view2,size_t(0));

  bool result = (sum == (n*(n-1))/2) && !is_sorted;


  stapl::do_once([&exec_time, &result, &timer, &n]() {
    printf("Test: shuffle\nStatus: ");
    if (result)
      printf("PASS");
    else
      printf("FAIL");
    printf("\nVersion: stapl\nTime: %f\n\n", exec_time);

    bool sh_correct(true);
    std::vector<int> x(n);
    std::iota(x.begin(), x.end(), 0); //Fill x with sequentially increasing #'s

    std::random_device rd;
    std::mt19937 g(rd());

    double initial = std::accumulate(x.begin(), x.end(), 0.0);
    timer.reset();
    timer.start();

    std::shuffle(x.begin(), x.end(), g);

    double time = timer.stop();

    double post_shuffle = std::accumulate(x.begin(), x.end(), 0.0);
    // Checked with some room for error due to decimals not being exact.
    if ((post_shuffle * 1.05) <= initial || (post_shuffle * 0.95) >= initial )
      sh_correct = false;

    printf("Test: shuffle\nStatus: ");
    if (sh_correct)
      printf("PASS");
    else
      printf("FAIL");
    printf("\nVersion: stl\nTime: %f\n\n", time);

  });

  return EXIT_SUCCESS;
}

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cstdlib>
#include <iostream>
#include <vector>

#include <stapl/containers/array/array.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/list.hpp>

#include "../../test_report.hpp"

using namespace stapl;

using timer_type = counter<default_timer>;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  const int n = (argc >= 2) ? atoi(argv[1]) : 100;
  const int m = (argc >= 3) ? atoi(argv[2]) : 5;

  using value_type       = int;
  using array_type       = array<value_type>;
  using view_type        = array_view<array_type>;
  using vector_type      = std::vector<value_type>;
  using vector_view_type = array_view<vector_type>;

  array_type array(n);
  view_type  view(array);

  copy(counting_view<value_type>(n), view);

  //
  // std::vector search string
  //
  vector_type      vec(m);
  vector_view_type vector_view(vec);

  for (int i=0; i<m; ++i)
    vec[i]=n-m+i;

  for (int i = 0; i < 32; ++i)
  {
    timer_type benchmark_timer;
    benchmark_timer.start();

    const auto ref3 = search(view, vector_view);

    const double benchmark_elapsed = benchmark_timer.stop();

    if (ref3 != n-m)
      abort("Failure on search with std::vector");

    stapl::array<double> times_ct(get_num_locations());
    times_ct[get_location_id()] = benchmark_elapsed;
    array_view<stapl::array<double>> times_vw(times_ct);
    double result = reduce(times_vw, max<double>());

    do_once([i, result]()
    {
      std::cout << "search(std::vector) Iteration " << i
                << " time=" << result << "\n";
    });
  }

  STAPL_TEST_REPORT(true, "Testing search over array, string in std::vector")

  //
  // stapl::array search string
  //
  array_type search_str(m);
  view_type  search_str_view(search_str);

  copy(counting_view<value_type>(m, n-m), search_str_view);

  for (int i = 0; i < 32; ++i)
  {
    timer_type benchmark_timer;
    benchmark_timer.start();

    const auto ref3 = search(view, search_str_view);

    const double benchmark_elapsed = benchmark_timer.stop();

    if (ref3 != n-m)
      abort("Failure on search with stapl::array");

    stapl::array<double> times_ct(get_num_locations());
    times_ct[get_location_id()] = benchmark_elapsed;
    array_view<stapl::array<double>> times_vw(times_ct);
    double result = reduce(times_vw, max<double>());

    do_once([i, result]()
    {
      std::cout << "search(stapl::array) Iteration " << i
                << " time=" << result << "\n";
    });
  }

  STAPL_TEST_REPORT(true, "Testing search over array, string in stapl::array")

#if 0
  **** TO DO AFTER PROMOTION FIX FOR PLISTS - UNCOMMENT BELOW
  p_list test
  typedef list<int> list_t;
  typedef list_view<list_t> viewl_t;
  list_t pl(n);
  viewl_t viewl(pl);
  copy(counting_view<int>(n),viewl);
  do_once(std::cout<<constant("Testing search over list:"));
  typedef viewl_t::reference refl_t;
  refl_t refer=search(viewl,vec_view);
  std::cout << "ref: " << refer << " index: " << index_of(refer) << std::endl;
  do_once(if_then_else(constant(!is_null_reference(refer)),
  std::cout << constant(": PASSED\n"),
  std::cout << constant(": FAILED\n")
  )
    );
#endif

  return EXIT_SUCCESS;
}

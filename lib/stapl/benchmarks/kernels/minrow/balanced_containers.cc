/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


//////////////////////////////////////////////////////////////////////
/// @file
/// Benchmark for nested parallelism. A @ref stapl::array object is created that
/// contains other @ref stapl::array objects that are each filled with
/// monotonically increasing integral values each. The parent @ref stapl::array
/// is as big as the number of locations the application runs on.
///
/// The application is invoked with
/// @code
/// mpirun -np num-processes ./balanced_containers parent-array-size child-array-size
/// @endcode
///
/// Two kernels are benchmarked:
/// -# Increasing all the values of the leaf @ref stapl::array objects
///    (@ref increment_all) and
/// -# Finding the minimum value in each leaf @ref stapl::array (@ref min_row).
//////////////////////////////////////////////////////////////////////

#include <stapl/containers/array/array.hpp>
#include <boost/lexical_cast.hpp>
#include "common_algorithms.hpp"
#include <iostream>


static bool print = false;


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using stapl::for_each;

  if (argc!=3) {
    std::cerr << "Usage: " << argv[0]
              << " #outer-container-size #inner-container-size\n";
    return EXIT_FAILURE;
  }

  const unsigned int N1 = boost::lexical_cast<unsigned int>(argv[1]);
  const unsigned int N2 = boost::lexical_cast<unsigned int>(argv[2]);

  if (N1==0 || N2==0) {
    std::cerr << argv[0] << ": container is empty.\n";
    return EXIT_FAILURE;
  }

  print = N1 <= 30 && N2 <= 30;

  // container and result container
  typedef int                      value_type;
  typedef stapl::array<value_type> array_type;
  typedef stapl::array<array_type> composed_array_type;

  array_type inner(N2);
  composed_array_type pa(N1, inner, stapl::policy::here());
  array_type res(pa.size());

  // views
  typedef stapl::array_view<composed_array_type> data_view_type;
  typedef stapl::array_view<array_type>          result_view_type;

  data_view_type vw(pa);
  result_view_type vw_res(res);

  // generate data
  stapl::map_func(set_val(), vw);

  if (stapl::get_location_id()==0) {
    std::cout << "container_size\t" << N1 << ' ' << N2 << '\n'
              << "parallelism   \t";
    print_parallelism(std::cout);
    std::cout << std::endl;
  }
  stapl::rmi_fence();

  if (!print) {
    // profiler based

    // increment_all
    {
      increment_all_prof<data_view_type> pr(vw, argc, argv);
      pr.collect_profile();
      pr.report();
      stapl::rmi_fence();
    }

    // min_row
    {
      min_row_prof<data_view_type, result_view_type> pr(vw, vw_res, argc, argv);
      pr.collect_profile();
      pr.report();
      stapl::rmi_fence();
    }
  }
  else {
    increment_all(vw);

    // print data
    if (stapl::get_location_id() == 0)
      std::cout << "----- array<array<int>> ---" << std::endl;
    stapl::rmi_fence();
    stapl::for_each(vw, print_pc());
    stapl::rmi_fence();

    // compute min_row() and store in parray<int>
    stapl::counter<stapl::default_timer> tmr;
    tmr.start();
    min_row(vw, vw_res);
    const double elapsed = tmr.stop();

    // print results
    if (stapl::get_location_id()==0) {
      std::cout << "min_row(): " << elapsed << "secs" << std::endl;
      std::cout << "----- Results after min_row(array<array<int>> ---"
                << std::endl;
    }
    stapl::rmi_fence();
    print_pc()(res);
    stapl::rmi_fence();
  }

  return EXIT_SUCCESS;
}

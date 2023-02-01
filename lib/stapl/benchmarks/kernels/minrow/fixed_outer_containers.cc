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
/// contains other @ref stapl::array objects that are each illed with
/// monotonically increasing integral values each. The parent @ref stapl::array
/// has a fixed number of inner containers, defined at runtime.
///
/// The application is invoked with
/// @code
/// mpirun -np num-processes ./fixed_outer_containers total-number-elements number-inner-containers
/// @endcode
///
/// Three kernels are benchmarked:
/// -# Increasing all the values of the inner @ref stapl::array objects
///    (@ref increment_all) and
/// -# Finding the minimum value in each inner @ref stapl::array (@ref min_row).
/// -# Finding the minimum value overall (@ref min_elem).
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
              << " #total-elements #inner-containers\n";
    return EXIT_FAILURE;
  }

  const unsigned int N = boost::lexical_cast<unsigned int>(argv[1]);
  print = (N <= 100);

  const unsigned int N1 = boost::lexical_cast<unsigned int>(argv[2]);
  const unsigned int N2 = (N / N1);

  if (N1==0 || N2==0) {
    std::cerr << argv[0] << ": container is empty.\n";
    return EXIT_FAILURE;
  }

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

    // min_elem
    {
      value_type v = value_type();
      min_elem_prof<data_view_type, value_type> pr(vw, v, argc, argv);
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
    double elapsed = tmr.stop();

    // print results
    if (stapl::get_location_id()==0) {
      std::cout << "min_row(): " << elapsed << "secs" << std::endl;
      std::cout << "----- Results after min_row(array<array<int>> ---"
                << std::endl;
    }
    stapl::rmi_fence();
    print_pc()(res);
    stapl::rmi_fence();

    // compute min_elem()
    tmr.reset();
    tmr.start();
    value_type v = min_elem<value_type>(vw);
    elapsed = tmr.stop();

    // print results
    if (stapl::get_location_id()==0) {
      std::cout << "min_elem(): " << elapsed << "secs" << std::endl;
      std::cout << "----- Results after min_elem(array<array<int>>) ---"
                << std::endl;
      std::cout << v << std::endl;
    }

    stapl::rmi_fence();
  }

  return EXIT_SUCCESS;
}

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
/// Benchmark that creates a @ref stapl::static_array and calls
/// @ref stapl::static_array::set_element for all elements from all locations.
///
/// This benchmark, while not indicative of a real application, stresses the
/// runtime system in term of both RMI processing speed and memory usage.
//////////////////////////////////////////////////////////////////////

#include <iostream>
#include <cstdlib>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/runtime/system.hpp>

using namespace stapl;

exit_code stapl_main(int argc, char* argv[])
{
  unsigned int N = 10;
  if (argc==2) {
    N = std::atoi(argv[1]);
  }
  if (get_location_id()==0) {
    std::cout << "N : " << N << std::endl;
  }

  static_array<unsigned int> a(N);

  counter<default_timer> c;

  rmi_fence(); // quiescence to start timing

  std::size_t memsize = runtime::get_available_physical_memory();

  c.start();
  for (unsigned int i = 0; i<N; ++i) {
    a.set_element(i, i);
  }
  rmi_fence(); // wait for all set_element calls to finish
  double time = c.stop();

  memsize -= runtime::get_available_physical_memory();

  if (get_location_id()==0) {
    std::cout << "Time (s):            " << time    << '\n'
              << "Process Memory (KB): " << double(memsize)/1024.0 << '\n';
  }

  return EXIT_SUCCESS;
}

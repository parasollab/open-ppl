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
/// @ref stapl::static_array::set_element for all elements from all locations
/// indirectly through RMIs.
///
/// This benchmark, while not indicative of a real application, stresses the
/// runtime system in term of both RMI processing speed and memory usage.
//////////////////////////////////////////////////////////////////////

#include <iostream>
#include <cstdlib>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/runtime/system.hpp>

using namespace stapl;

class A
: public p_object
{
private:
  unsigned int               N;
  static_array<unsigned int> a;

public:
  A(unsigned int n)
  : N(n),
    a(N)
  { }

  void left(void)
  {
    unsigned int end = N/2;
    for (unsigned int i = 0; i<end; ++i) {
      a.set_element(i, i);
    }
  }

  void right(void)
  {
    for (unsigned int i = N/2; i<N; ++i) {
      a.set_element(i, i);
    }
  }

  void here(void)
  {
    for (unsigned int i = 0; i<N; ++i) {
      a.set_element(i, i);
    }
  }
};


exit_code stapl_main(int argc, char* argv[])
{
  unsigned int N = 10;
  if (argc==2) {
    N = std::atoi(argv[1]);
  }
  if (get_location_id()==0) {
    std::cout << "N : " << N << std::endl;
  }

  A a(N);
#ifdef REMOTE_CALLS
  const unsigned int left = (a.get_location_id()==0)
                               ? a.get_num_locations() - 1
                               : a.get_location_id()-1;
  const unsigned int right = (a.get_location_id()+1) % a.get_num_locations();
#endif

  counter<default_timer> c;

  rmi_fence(); // quiescence before start timing

  std::size_t memsize = runtime::get_available_physical_memory();

  c.start();

#ifdef REMOTE_CALLS
  async_rmi(left, a.get_rmi_handle(), &A::left);
  async_rmi(right, a.get_rmi_handle(), &A::right);
#else
  async_rmi(a.get_location_id(), a.get_rmi_handle(), &A::here);
#endif

  rmi_fence(); // wait for async_rmi calls to finish

  double time = c.stop();
  memsize -= runtime::get_available_physical_memory();

  if (get_location_id()==0) {
    std::cout << "Time (s):            " << time    << '\n'
              << "Process Memory (KB): " << double(memsize)/1024.0 << '\n';
  }

  return EXIT_SUCCESS;
}

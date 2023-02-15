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
/// Benchmark that does a lot of RMI requests targeted at each one of the
/// locations.
///
/// This benchmark, while not indicative of a real application, stresses the
/// runtime system in term of both RMI processing speed and memory usage.
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <stapl/runtime/system.hpp>
#include <iostream>

class A
: public stapl::p_object
{
private:
  unsigned int m_next;

public:
  A(void)
  : m_next(get_location_id())
  { }

  unsigned int get_next_location(void)
  {
    m_next = (m_next+1)%get_num_locations();
    return m_next;
  }

  void foo(void)
  { }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  const unsigned int reqs_per_location = 1000000;
  unsigned int N = reqs_per_location * stapl::get_num_locations();

  A a;
  stapl::counter<stapl::default_timer> c;

  stapl::rmi_fence(); // quiescence before start timing

  std::size_t memsize = stapl::runtime::get_available_physical_memory();

  c.start();
  for (unsigned int i = 0; i<N; ++i) {
    stapl::async_rmi(a.get_next_location(), a.get_rmi_handle(), &A::foo);
  }
  stapl::rmi_fence(); // wait for all async_rmi calls to finish
  double time = c.stop();
  memsize -= stapl::runtime::get_available_physical_memory();

  if (stapl::get_location_id()==0) {
    std::cout << "Time (s):            " << time    << '\n'
              << "Process Memory (KB): " << double(memsize)/1024.0 << '\n';
  }

  return EXIT_SUCCESS;
}

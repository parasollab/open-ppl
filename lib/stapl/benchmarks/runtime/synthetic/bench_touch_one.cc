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
/// Benchmark where each location communicates with one other random location.
///
/// The kernel invokes RMIs from one location to a randomly selected location
/// that will trigger an RMI to another random location.
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#include <stapl/runtime/random_location_generator.hpp>
#include <string>

// Avoids combining
class A
: public stapl::p_object
{
private:
  unsigned int                     m_next;
  stapl::random_location_generator m_gen;

public:
  A(void)
  : m_next((this->get_location_id()+1)%this->get_num_locations())
  { }

  void foo(unsigned int N)
  {
    if (N==0) {
      return;
    }
    unsigned int i = m_gen();
    stapl::async_rmi(i, this->get_rmi_handle(), &A::foo, N-1);
  }

  void call_everyone(void)
  {
    foo(this->get_num_locations());
    stapl::rmi_fence(); // wait for all async_rmi calls to finish
  }
};


// Kernel that benchmarks async_rmi()
struct async_bench_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("touch one: "); }

  A obj;

  void operator()(void)
  { obj.call_everyone(); }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

  {
    async_bench_wf wf;
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}

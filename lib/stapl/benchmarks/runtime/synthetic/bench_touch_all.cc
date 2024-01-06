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
/// Benchmark where each location communicates with all other locations.
///
/// The kernel invokes RMIs from all locations to all locations that will
/// trigger RMIs to all locations. It is a very heavy communication pattern.
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#include <string>

// Avoids combining
class A
: public stapl::p_object
{
public:
  void foo(unsigned int N)
  {
    if (N==0) {
      return;
    }
    for (unsigned int i=0; i<this->get_num_locations(); ++i) {
      stapl::async_rmi(i, this->get_rmi_handle(), &A::foo, N-1);
    }
  }

  void call_everyone(void)
  {
    foo(this->get_num_locations());
    stapl::rmi_fence(); // wait for async_rmi calls to finish
  }
};


// Kernel that benchmarks async_rmi()
struct async_bench_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("touch all: "); }

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

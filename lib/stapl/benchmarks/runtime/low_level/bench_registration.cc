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
/// Benchmark that times registration.
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"

using namespace stapl;

// Kernel that benchmarks rmi_handle registration
struct rmi_handle_registration_wf
{
  typedef void result_type;

  result_type operator()(void) const
  {
    int i = {};
    rmi_handle h(&i);
  }

  static std::string name(void)
  { return std::string("rmi_handle()"); }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

  {
    rmi_handle_registration_wf wf;
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}

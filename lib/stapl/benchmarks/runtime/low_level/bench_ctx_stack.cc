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
/// Benchmark that times grabbing the stack vs increasing a thread local
/// variable.
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#include <stapl/runtime/concurrency/thread_local_storage.hpp>

STAPL_RUNTIME_THREAD_LOCAL(int, i)


// Kernel that benchmarks TLS
struct tls_wf
{
  typedef void result_type;

  result_type operator()(void) const
  { ++i.get(); }

  static std::string name(void)
  { return std::string("tls"); }
};


// Kernel that benchmarks this_context::get()
struct get_ctx_wf
{
  typedef void result_type;

  result_type operator()(void) const
  { stapl::runtime::this_context::get(); }

  static std::string name(void)
  { return std::string("get ctx"); }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

  {
    tls_wf wf;
    p.benchmark(wf);
  }

  {
    get_ctx_wf wf;
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}

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
/// Benchmark that compares the overhead of forking through
/// @ref stapl::execute() vs OpenMP parallel.
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#include <omp.h>
#include <cstdlib>
#include <stdexcept>
#include <boost/lexical_cast.hpp>

static __attribute__((noinline)) void foo(int i, int j)
{
  if (i>j)
    throw std::logic_error("should not happen");
}


// Kernel that benchmarks openmp
struct omp_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("openmp        "); }

  void operator()(void)
  {
#pragma omp parallel
    {
      const int tid = omp_get_thread_num();
      const int nth = omp_get_num_threads();
      foo(tid, nth);
    }
  }
};


// Kernel that benchmarks openmp barrier
struct omp_barrier_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("openmp_barrier"); }

  void operator()(void)
  {
#pragma omp parallel
    {
      const int tid = omp_get_thread_num();
      const int nth = omp_get_num_threads();
#pragma omp barrier
      foo(tid, nth);
    }
  }
};


// Kernel that benchmarks execute()
struct exec_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("execute       "); }

  struct empty_wf
  {
    void operator()() const
    {
      const std::pair<int,int> p = stapl::get_location_info();
      foo(p.first, p.second);
    }
  };

  void operator()(void)
  { stapl::execute(empty_wf()); }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);
  std::string s = "threads " +
                  boost::lexical_cast<std::string>(omp_get_max_threads());
  p.add_text(s);
  char* policy = std::getenv("OMP_WAIT_POLICY");
  if (policy && std::strcmp("PASSIVE", policy)==0)
    p.add_text("suffix _passive");

  const unsigned int N = 1000;

  {
    omp_wf wf;
    p.warmup(wf);
    p.benchmark(wf, N);
  }

  {
    omp_barrier_wf wf;
    p.benchmark(wf, N);
  }

  {
    exec_wf wf;
    p.benchmark(wf, N);
  }

  return EXIT_SUCCESS;
}

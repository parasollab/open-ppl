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
/// @ref stapl::execute() for multiple levels.
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#include <boost/lexical_cast.hpp>

// Kernel that benchmarks execute()
template<unsigned int N>
struct execute_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("execute() ") + boost::lexical_cast<std::string>(N); }

  static void execute(unsigned int n)
  {
    if (n!=0)
      stapl::execute([n] { execute(n-1); });
  }

  void operator()(void)
  { 
    // capturing this even though it is not needed to avoid a gcc 4.7 error
    stapl::execute([this] { execute(N-1); });
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

  {
    execute_wf<1> wf;
    p.benchmark(wf);
  }

  {
    execute_wf<2> wf;
    p.benchmark(wf);
  }

  {
    execute_wf<3> wf;
    p.benchmark(wf);
  }

  {
    execute_wf<4> wf;
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}

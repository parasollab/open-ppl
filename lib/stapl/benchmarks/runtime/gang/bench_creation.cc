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
/// Benchmark for overhead of creating a gang of 1 location through
/// @ref stapl::execute().
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"

struct empty_wf
{
  void operator()(void) const
  { }
};

class A
: public stapl::p_object
{
public:
  void no_make(void)
  { }

  void make(void)
  { stapl::execute(empty_wf()); }
};

// Kernel that benchmarks no gang creation through an async_rmi
struct no_creation_bench_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("no_create"); }

  A obj;

  result_type operator()(void)
  {
    if (stapl::get_location_id()==0) {
      stapl::async_rmi(0, obj.get_rmi_handle(), &A::no_make);
    }
  }
};


// Kernel that benchmarks gang creation through an async_rmi
struct creation_bench_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("create"); }

  A obj;

  result_type operator()(void)
  {
    if (stapl::get_location_id()==0) {
      stapl::async_rmi(0, obj.get_rmi_handle(), &A::make);
    }
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

  {
    no_creation_bench_wf wf;
    p.benchmark(wf);
  }

  {
    creation_bench_wf wf;
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}

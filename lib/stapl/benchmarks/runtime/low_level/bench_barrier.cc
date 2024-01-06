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
/// Benchmark for shared-memory barrier and reduction vs OpenMP barrier.
///
/// The following are benchmarked:
/// -# OpenMP barrier
/// -# @ref stapl::runtime::fuzzy_barrier
/// -# @ref stapl::runtime::reduction
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#include <stapl/runtime/concurrency/fuzzy_barrier.hpp>
#include <stapl/runtime/concurrency/reduction.hpp>
#include <functional>
#include <memory>

void foo(void)
{ }


struct dummy_p_object
: public stapl::p_object
{ };


// Kernel that benchmarks execute() with omp barrier
struct execute_omp_wf
{
  static std::string name(void)
  { return std::string("execute(omp) "); }

  static void execute(void)
  {
    for (int i=0; i<10000; ++i) {
#pragma omp barrier
    }
  }

  void operator()(void)
  { stapl::execute(execute); }
};


// Kernel that benchmarks execute() with my barrier
struct execute_my_wf
{
  static std::string name(void)
  { return std::string("execute(my) "); }

  static void execute(stapl::rmi_handle::const_reference const& r)
  {
    using barrier_type = stapl::runtime::fuzzy_barrier;

    auto& ctx      = stapl::runtime::this_context::get();
    auto& l        = ctx.get_location_md();
    auto& g        = l.get_gang_md();
    const auto lid = l.local_index();
    auto p =
      g.get_shared_object<barrier_type>(r, g.local_size());
    for (int i=0; i<10000; ++i) {
      p->wait(lid, foo);
    }
  }

  void operator()(void)
  {
    dummy_p_object o;
    stapl::execute(std::bind(execute, o.get_rmi_handle()));
  }
};


// Kernel that benchmarks execute() with my reduction
struct execute_red_wf
{
  static std::string name(void)
  { return std::string("execute(red) "); }

  static void execute(stapl::rmi_handle::const_reference const& r)
  {
    using op_type = stapl::runtime::reduction<int, std::plus<int>>;

    auto& ctx      = stapl::runtime::this_context::get();
    auto& l        = ctx.get_location_md();
    auto& g        = l.get_gang_md();
    const auto lid = l.local_index();
    auto p         = g.get_shared_object<op_type>(r, g.local_size());
    for (int i=0; i<10000; ++i) {
      if ((*p)(lid, 1))
        p->get();
#pragma omp barrier
    }
  }

  void operator()(void)
  {
    dummy_p_object o;
    stapl::execute(std::bind(execute, o.get_rmi_handle()));
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

  {
    execute_omp_wf wf;
    p.benchmark(wf);
  }

  {
    execute_my_wf wf;
    p.benchmark(wf);
  }

  {
    execute_red_wf wf;
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}

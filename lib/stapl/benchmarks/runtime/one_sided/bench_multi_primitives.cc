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
/// Benchmark for @ref stapl::async_rmi() to all locations.
///
/// The following are benchmarked:
/// -# @ref MPI_Bcast()
/// -# @ref stapl::async_rmi() on all locations.
/// -# @ref stapl::unordered::async_rmi() on all locations.
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#ifndef STAPL_DONT_USE_MPI
# include <mpi.h>
#endif
#include <vector>

#ifndef STAPL_DONT_USE_MPI

// Kernel that benchmarks MPI_Bcast()
struct MPI_Bcast_bench_wf
{
  typedef void result_type;

  MPI_Comm          comm;
  std::vector<char> array;

  MPI_Bcast_bench_wf(const std::size_t s)
  : comm(MPI_COMM_NULL),
    array(s)
  {
    MPI_Comm_dup(MPI_COMM_WORLD, &comm);
  }

  ~MPI_Bcast_bench_wf(void)
  {
    MPI_Barrier(comm);
    MPI_Comm_free(&comm);
  }

  result_type operator()(void)
  {
    MPI_Bcast(&array[0], array.size(), MPI_BYTE, 0, comm);
  }

  std::string name(void)
  {
    return (std::string("MPI_Bcast(") +
            boost::lexical_cast<std::string>(array.size()) +
            std::string(")"));
  }
};

#endif


// Avoids combining
class A
: public stapl::p_object
{
private:
  std::vector<char> array;

public:
  A(const std::size_t s)
  : array(s)
  { this->advance_epoch(); }

  void foo(std::vector<char> const&)
  { }

  void call_multi_async_rmi(void)
  {
    if (this->get_location_id()==0) {
      stapl::async_rmi(stapl::all_locations, this->get_rmi_handle(),
                       &A::foo, array);
    }
  }

  void call_multi_async_rmi_unordered(void)
  {
    if (this->get_location_id()==0) {
      stapl::unordered::async_rmi(stapl::all_locations, this->get_rmi_handle(),
                                  &A::foo, array);
    }
  }

  std::size_t size(void) const
  { return array.size(); }
};


// Kernel that benchmarks multi_async_rmi()
struct multi_async_bench_wf
{
  typedef void result_type;

  A obj;

  multi_async_bench_wf(const std::size_t s)
  : obj(s)
  { }

  void operator()(void)
  { obj.call_multi_async_rmi(); }

  std::string name(void)
  {
    return (std::string("async_rmi(all_locations") +
            boost::lexical_cast<std::string>(obj.size()) +
            std::string(")"));
  }
};


// Kernel that benchmarks unordered::multi_async_rmi()
struct multi_async_unordered_bench_wf
{
  typedef void result_type;

  A obj;

  multi_async_unordered_bench_wf(const std::size_t s)
  : obj(s)
  { }

  void operator()(void)
  { obj.call_multi_async_rmi_unordered(); }

  std::string name(void)
  {
    return (std::string("unordered::async_rmi(all_locations") +
            boost::lexical_cast<std::string>(obj.size()) +
            std::string(")"));
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  const std::size_t start = 1024;
  const std::size_t end   = (8 * start);
  const std::size_t step  = 2;

  stapl::runtime_profiler<> p(argc, argv);

  for (std::size_t i = start; i <= end; i*=step) {
#ifndef STAPL_DONT_USE_MPI
    if (!stapl::is_in_mixed_mode()) {
      MPI_Bcast_bench_wf wf(i);
      p.benchmark(wf);
    }
#endif

    {
      multi_async_bench_wf wf(i);
      p.benchmark(wf);
    }

    {
      multi_async_unordered_bench_wf wf(i);
      p.benchmark(wf);
    }
  }

  return EXIT_SUCCESS;
}

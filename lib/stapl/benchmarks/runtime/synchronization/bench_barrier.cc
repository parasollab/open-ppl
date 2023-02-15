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
/// Benchmark for barriers only for distributed memory.
///
/// The following are benchmarked:
/// -# @ref MPI_Barrier()
/// -# @ref MPI_Ibarrier()
/// -# @ref stapl::rmi_barrier()
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#ifndef STAPL_DONT_USE_MPI
# include <mpi.h>
#endif


#ifndef STAPL_DONT_USE_MPI

// Kernel that benchmarks MPI_Barrier()
struct MPI_Barrier_bench_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("MPI_Barrier"); }

  MPI_Comm comm;

  MPI_Barrier_bench_wf(void)
  : comm(MPI_COMM_NULL)
  {
    MPI_Comm_dup(MPI_COMM_WORLD, &comm);
  }

  ~MPI_Barrier_bench_wf(void)
  {
    MPI_Barrier(comm);
    MPI_Comm_free(&comm);
  }

  result_type operator()(void) const
  { MPI_Barrier(comm); }
};


#ifdef USE_NBC

// Kernel that benchmarks MPI_Ibarrier()
struct MPI_Ibarrier_bench_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("MPI_Ibarrier"); }

  MPI_Comm    comm;
  MPI_Request handle;

  MPI_Ibarrier_bench_wf(void)
  : comm(MPI_COMM_NULL),
    handle(MPI_REQUEST_NULL)
  {
    MPI_Comm_dup(MPI_COMM_WORLD, &comm);
  }

  ~MPI_Ibarrier_bench_wf(void)
  {
    MPI_Barrier(comm);
    MPI_Comm_free(&comm);
  }

  result_type operator()(void)
  {
    MPI_Ibarrier(comm, &handle);
    MPI_Wait(&handle, MPI_STATUS_IGNORE);
  }
};

#endif

#endif // STAPL_DONT_USE_MPI


// Kernel that benchmarks stapl::rmi_barrier()
struct barrier_bench_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("rmi_barrier"); }

  result_type operator()(void) const
  { stapl::rmi_barrier(); }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

#ifndef STAPL_DONT_USE_MPI
  if (!stapl::is_in_mixed_mode()) {
    {
      MPI_Barrier_bench_wf wf;
      p.benchmark(wf);
    }

#ifdef USE_NBC
    {
      MPI_Ibarrier_bench_wf wf;
      p.benchmark(wf);
    }
#endif
  }
#endif

  {
    barrier_bench_wf wf;
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}

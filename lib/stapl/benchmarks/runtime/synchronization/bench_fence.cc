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
/// Benchmark for @ref stapl::rmi_fence() only for distributed memory.
///
/// The following are benchmarked:
/// -# @ref stapl::rmi_fence()
/// -# Simulated @ref stapl::rmi_fence() with @ref MPI_Allreduce().
/// -# Simulated @ref stapl::rmi_fence() with @ref stapl::rmi_barrier() and
///    @ref MPI_Allreduce().
/// -# Simulated @ref stapl::rmi_fence() with @ref MPI_Iallreduce().
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#ifndef STAPL_DONT_USE_MPI
# include <mpi.h>
#endif


#ifndef STAPL_DONT_USE_MPI

// Kernel that benchmarks a simulated rmi_fence() using MPI_Allreduce()
struct sim_rmi_fence_bench_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("MPI_Allreduce"); }

  MPI_Comm comm;

  sim_rmi_fence_bench_wf(void)
  : comm(MPI_COMM_NULL)
  { MPI_Comm_dup(MPI_COMM_WORLD,  &comm); }

  ~sim_rmi_fence_bench_wf(void)
  {
    MPI_Barrier(comm);
    MPI_Comm_free(&comm);
  }

  result_type operator()(void) const
  {
    int req = 0;
    int phase = 1;
    do {
      int recv = 0;
      MPI_Allreduce(&recv, &req, 1, MPI_INT, MPI_SUM, comm);
      switch (phase) {
      case 1:
        if (recv==0) phase = 2;
        break;
      case 2:
        if (recv!=0) phase = 1;
        else phase = 3;
        break;
      }
    } while (phase!=3);
  }
};


// Kernel that simulates stapl::rmi_fence() with barrier + MPI_Allreduce()
struct sim_rmi_fence_2_bench_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("rmi_barrier+MPI_Allreduce"); }

  MPI_Comm comm;

  sim_rmi_fence_2_bench_wf(void)
  : comm(MPI_COMM_WORLD)
  { MPI_Comm_dup(MPI_COMM_WORLD,  &comm); }

  ~sim_rmi_fence_2_bench_wf(void)
  {
    MPI_Barrier(comm);
    MPI_Comm_free(&comm);
  }

  result_type operator()(void) const
  {
    int req = 0;
    int phase = 1;
    do {
      int recv = 0;
      stapl::rmi_barrier();
      MPI_Allreduce(&recv, &req, 1, MPI_INT, MPI_SUM, comm);
      switch (phase) {
      case 1:
        if (recv==0) phase = 2;
        break;
      case 2:
        if (recv!=0) phase = 1;
        else phase = 3;
        break;
      }
    } while (phase!=3);
  }
};


#ifdef USE_NBC

// Kernel that benchmarks a simulated rmi_fence() using MPI_Iallreduce()
struct sim_rmi_fence_3_bench_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("NBC_Iallreduce"); }

  MPI_Comm    comm;
  MPI_Request handle;

  sim_rmi_fence_3_bench_wf(void)
  : comm(MPI_COMM_NULL),
    handle(MPI_REQUEST_NULL)
  { MPI_Comm_dup(MPI_COMM_WORLD,  &comm); }

  ~sim_rmi_fence_3_bench_wf(void)
  {
    MPI_Barrier(comm);
    MPI_Comm_free(&comm);
  }

  result_type operator()(void)
  {
    int req = 0;
    int phase = 1;
    do {
      int recv = 0;
      MPI_Iallreduce(&recv, &req, 1, MPI_INT, MPI_SUM, comm, &handle);
      MPI_Wait(&handle, MPI_STATUS_IGNORE);
      switch (phase) {
      case 1:
        if (recv==0) phase = 2;
        break;
      case 2:
        if (recv!=0) phase = 1;
        else phase = 3;
        break;
      }
    } while (phase!=3);
  }
};

#endif

#endif // STAPL_DONT_USE_MPI


// Kernel that benchmarks stapl::rmi_fence()
struct fence_bench_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("rmi_fence"); }

  result_type operator()(void) const
  { stapl::rmi_fence(); }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

#ifndef STAPL_DONT_USE_MPI
  if (!stapl::is_in_mixed_mode()) {
    {
      sim_rmi_fence_bench_wf wf;
      p.benchmark(wf);
    }

    {
      sim_rmi_fence_2_bench_wf wf;
      p.benchmark(wf);
    }

#ifdef USE_NBC
    {
      sim_rmi_fence_3_bench_wf wf;
      p.benchmark(wf);
    }
#endif
  }
#endif

  {
    fence_bench_wf wf;
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}

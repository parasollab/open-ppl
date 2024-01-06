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
/// Benchmark for one-sided reduction.
///
/// The following are benchmarked:
/// -# @ref MPI_Reduce() with commutative operator.
/// -# @ref stapl::sync_reduce_rmi() with commutative operator.
/// -# @ref stapl::unordered::sync_reduce_rmi() with commutative operator.
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#ifndef STAPL_DONT_USE_MPI
# include <mpi.h>
#endif


#ifndef STAPL_DONT_USE_MPI

// Kernel that benchmarks MPI_Reduce() - commutative
struct MPI_Reduce_bench_wf
{
  typedef void result_type;

  static std::string name(void)
  { return std::string("MPI_Reduce(C)"); }

  MPI_Comm comm;

  MPI_Reduce_bench_wf(void)
  : comm(MPI_COMM_NULL)
  { MPI_Comm_dup(MPI_COMM_WORLD,  &comm); }

  ~MPI_Reduce_bench_wf(void)
  {
    MPI_Barrier(comm);
    MPI_Comm_free(&comm);
  }

  result_type operator()(void) const
  {
    double val = 1.0;
    double res = 0.0;
    MPI_Reduce(&val, &res, 1, MPI_DOUBLE, MPI_SUM, 0, comm);
  }
};

#endif


template<typename T>
class A
: public stapl::p_object
{
public:
  T get(void) const
  { return 1.0; }
};


// Kernel that benchmarks stapl::sync_reduce_rmi() - commutative
struct reduce_rmi_bench_wf
{
  typedef void   result_type;
  typedef double value_type;

  static std::string name(void)
  { return std::string("sync_reduce_rmi(C)"); }

  A<value_type> obj;

  result_type operator()(void) const
  {
    if (obj.get_location_id()==0) {
      stapl::sync_reduce_rmi(std::plus<value_type>(), obj.get_rmi_handle(),
                             &A<value_type>::get);
    }
  }

};


// Kernel that benchmarks stapl::unordered::sync_reduce_rmi() - commutative
struct reduce_rmi_unordered_bench_wf
{
  typedef void   result_type;
  typedef double value_type;

  static std::string name(void)
  { return std::string("unordered::sync_reduce_rmi(C)"); }

  A<value_type> obj;

  result_type operator()(void) const
  {
    if (obj.get_location_id()==0) {
      stapl::unordered::sync_reduce_rmi(std::plus<value_type>(),
                                        obj.get_rmi_handle(),
                                        &A<value_type>::get);
    }
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

#ifndef STAPL_DONT_USE_MPI
  if (!stapl::is_in_mixed_mode()) {
    MPI_Reduce_bench_wf wf;
    p.benchmark(wf);
  }
#endif

  {
    reduce_rmi_bench_wf wf;
    p.benchmark(wf);
  }

  {
    reduce_rmi_unordered_bench_wf wf;
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}

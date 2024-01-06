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
/// Benchmark for broadcasts only for distributed memory.
///
/// The following are benchmarked:
/// -# @ref MPI_Bcast()
/// -# @ref MPI_Ibcast()
/// -# @ref stapl::broadcast_rmi()
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#ifndef STAPL_DONT_USE_MPI
# include <mpi.h>
#endif
#include <boost/lexical_cast.hpp>
#include <vector>


#ifndef STAPL_DONT_USE_MPI

// Kernel that benchmarks MPI_Bcast()
struct MPI_Bcast_bench_wf
{
  typedef void result_type;

  MPI_Comm    comm;
  int         numprocs;
  std::size_t size;
  char*       array;

  MPI_Bcast_bench_wf(const std::size_t s)
  : comm(MPI_COMM_NULL),
    size(s),
    array(new char[s])
  {
    MPI_Comm_dup(MPI_COMM_WORLD, &comm);
    MPI_Comm_size(comm, &numprocs);
  }

  ~MPI_Bcast_bench_wf(void)
  {
    delete[] array;
    MPI_Barrier(comm);
    MPI_Comm_free(&comm);
  }

  result_type operator()(void)
  {
    for (int i=0; i<numprocs; ++i) {
      MPI_Bcast(array, size*sizeof(*array), MPI_BYTE, i, comm);
    }
  }

  std::string name(void) const
  {
    return (std::string("MPI_Bcast(") +
            boost::lexical_cast<std::string>(size) + std::string(")"));
  }
};


#ifdef USE_NBC

// Kernel that benchmarks MPI_Ibcast()
struct MPI_Ibcast_bench_wf
{
  typedef void result_type;

  MPI_Comm    comm;
  int         numprocs;
  std::size_t size;
  char*       array;
  MPI_Request handle;

  MPI_Ibcast_bench_wf(const std::size_t s)
  : comm(MPI_COMM_NULL),
    size(s),
    array(new char[s]),
    handle(MPI_REQUEST_NULL)
  {
    MPI_Comm_dup(MPI_COMM_WORLD, &comm);
    MPI_Comm_size(comm, &numprocs);
  }

  ~MPI_Ibcast_bench_wf(void)
  {
    delete[] array;
    MPI_Barrier(comm);
    MPI_Comm_free(&comm);
  }

  result_type operator()(void)
  {
    for (int i=0; i<numprocs; ++i) {
      MPI_Ibcast(array, size*sizeof(*array), MPI_BYTE, i, comm, &handle);
      MPI_Wait(&handle, MPI_STATUS_IGNORE);
    }
  }

  std::string name(void) const
  {
    return (std::string("MPI_Ibcast(") +
            boost::lexical_cast<std::string>(size) + std::string(")"));
  }
};

#endif

#endif // STAPL_DONT_USE_MPI


class A
: public stapl::p_object
{
public:
  void id(std::vector<char> const&)
  { }
};


// Kernel that benchmarks broadcast_rmi()
struct broadcast_rmi_bench_wf
{
  typedef void result_type;

  std::vector<char> vec;
  A obj;

  broadcast_rmi_bench_wf(const std::size_t s)
  { vec.resize(s); }

  result_type operator()(void)
  {
    const unsigned int lid = obj.get_location_id();
    for (unsigned int i=0; i<obj.get_num_locations(); ++i)
      if (i==lid)
        stapl::broadcast_rmi(stapl::root_location, obj.get_rmi_handle(),
                             &A::id, vec).get();
      else
        stapl::broadcast_rmi(lid, &A::id).get();
  }

  std::string name(void) const
  {
    return (std::string("broadcast_rmi(") +
            boost::lexical_cast<std::string>(vec.size()) + std::string(")"));
  }
};


const std::size_t SIZE_START = 1024;
const std::size_t SIZE_END   = 3 * SIZE_START;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  stapl::runtime_profiler<> p(argc, argv);

#ifndef STAPL_DONT_USE_MPI
  if (!stapl::is_in_mixed_mode()) {
    for (std::size_t i = SIZE_START; i <= SIZE_END; i*=2) {
      MPI_Bcast_bench_wf wf(i);
      p.benchmark(wf);
    }

#ifdef USE_NBC
    for (std::size_t i = SIZE_START; i <= SIZE_END; i*=2) {
      MPI_Ibcast_bench_wf wf(i);
      p.benchmark(wf);
    }
#endif
  }
#endif

  for (std::size_t i = SIZE_START; i <= SIZE_END; i*=2) {
    broadcast_rmi_bench_wf wf(i);
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}

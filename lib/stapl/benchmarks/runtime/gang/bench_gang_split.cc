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
/// Benchmark for splitting a gang into two new ones.
///
/// The following are benchmarked:
/// -# @ref MPI_Comm_split()
/// -# @ref stapl::gang
//////////////////////////////////////////////////////////////////////

#include "../benchmark.h"
#ifndef STAPL_DONT_USE_MPI
# include <mpi.h>
#endif
#include <algorithm>
#include <numeric>
#include <boost/lexical_cast.hpp>


//#define USE_BARRIER


#ifndef STAPL_DONT_USE_MPI

// Kernel that benchmarks MPI_Comm_split
struct mpi_comm_split_wf
{
  typedef void result_type;

  MPI_Comm m_comm;
  int      m_size;

  mpi_comm_split_wf(void)
  : m_comm(MPI_COMM_NULL),
    m_size(0)
  {
    MPI_Comm_dup(MPI_COMM_WORLD, &m_comm);
    MPI_Comm_size(m_comm, &m_size);
  }

  result_type operator()(void)
  {
    int rank = 0;
    MPI_Comm_rank(m_comm, &rank);
    int color = 0;
    if (rank>=(m_size/2))
      color = 1;
    MPI_Comm newcomm = MPI_COMM_NULL;
    MPI_Comm_split(m_comm, color, 0, &newcomm); // collective over m_comm
#ifdef USE_BARRIER
    MPI_Barrier(newcomm);
#endif
    MPI_Comm_free(&newcomm);
  }

  std::string name(void) const
  {
    return (std::string("MPI_Comm_split ") +
            boost::lexical_cast<std::string>(m_size));
  }
};

#endif


struct block_mapper
{
  typedef unsigned int result_type;

  typedef std::tuple<unsigned int> member_types;

  const unsigned int m_block_size;

  explicit block_mapper(const unsigned int block_size)
  : m_block_size(block_size)
  { }

  unsigned int operator()(unsigned int n) const
  { return n % m_block_size; }

  bool operator==(block_mapper const& rhs) const
  { return (m_block_size==rhs.m_block_size); }
};


struct block_reverse_mapper
{
  typedef unsigned int result_type;

  typedef std::tuple<unsigned int> member_types;

  const unsigned int m_offset;

  explicit block_reverse_mapper(const unsigned int offset)
  : m_offset(offset)
  { }

  unsigned int operator()(unsigned int n) const
  { return (m_offset + n); }

  bool operator==(block_reverse_mapper const& rhs) const
  { return (m_offset==rhs.m_offset); }
};


// Kernel that benchmarks stapl::gang
struct gang_split_wf
{
  typedef void result_type;

  result_type operator()(void)
  {
    const unsigned int lid    = stapl::get_location_id();
    const unsigned int size   = stapl::get_num_locations()/2;
    const unsigned int offset = (lid<size ? 0 : size);
    stapl::gang g(size,
                  block_mapper(size),
                  block_reverse_mapper(offset));
#ifdef USE_BARRIER
    stapl::rmi_barrier();
#endif
  }

  std::string name(void) const
  {
    const unsigned int nth =
      (stapl::get_num_locations() / stapl::get_num_processes());
    const std::string s =
      (std::string("gang(") + boost::lexical_cast<std::string>(nth) +
       std::string("threads/process) "));

    return (s + boost::lexical_cast<std::string>(stapl::get_num_locations()));
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using namespace stapl;

  runtime_profiler<> p(argc, argv);

#ifndef STAPL_DONT_USE_MPI
  if (!is_in_mixed_mode()) {
    mpi_comm_split_wf wf;
    p.benchmark(wf);
  }
#endif

  {
    gang_split_wf wf;
    p.benchmark(wf);
  }

  return EXIT_SUCCESS;
}

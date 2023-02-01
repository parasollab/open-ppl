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
/// Benchmark to evaluate the memory footprint of MPI and OpenMP+MPI.
//////////////////////////////////////////////////////////////////////

#ifndef STAPL_DONT_USE_MPI
# include <mpi.h>
#endif
#ifdef _OPENMP
# include <omp.h>
#endif
#include <iostream>

int main(int argc, char *argv[])
{
  int rank = 0, size = 1;

#ifndef STAPL_DONT_USE_MPI
  MPI_Init(&argc, &argv);
  MPI_Comm_size(MPI_COMM_WORLD, &size);
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);
#endif

#ifdef _OPENMP
#pragma omp parallel
  {
    int tid = omp_get_thread_num();
    if (rank==0 && tid==0) {
      std::cout << "kernel    " << "MPI+OpenMP"                   << '\n'
                << "elements  " << (size * omp_get_num_threads()) << '\n'
                << "processes " << size                           << '\n'
                << "threads   " << omp_get_num_threads()
                << std::endl;
    }
  }
#else
  if (rank==0) {
    std::cout << "kernel    " << "MPI" << '\n'
              << "elements " << size   << '\n'
              << "processes " << size  << std::endl;
  }
#endif

#ifndef STAPL_DONT_USE_MPI
  MPI_Barrier(MPI_COMM_WORLD);
  MPI_Finalize();
#endif

  return 0;
}

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
/// Naive Hybrid MPI+OpenMP implementation of http://www.mcs.anl.gov/research/projects/mpi/tutorial/mpiexmpl/src/jacobi/C/main.html
//////////////////////////////////////////////////////////////////////

#include <iostream>
#include <cfloat>
#include <cmath>
#include <vector>
#include <mpi.h>
#ifdef _OPENMP
# include <omp.h>
#endif

template<typename T>
class matrix
{
private:
  std::size_t    m_nrows;
  std::size_t    m_ncols;
  std::vector<T> m_data;

public:
  matrix(std::size_t nrows, std::size_t ncols)
  : m_nrows(nrows), m_ncols(ncols), m_data(ncols * nrows)
  { }

  T const& operator()(std::size_t row, std::size_t ncol) const noexcept
  { return m_data[row * m_ncols + ncol]; }

  T& operator()(std::size_t row, std::size_t ncol) noexcept
  { return m_data[row * m_ncols + ncol]; }

  T const* operator[](std::size_t row) const noexcept
  { return &m_data[row * m_ncols]; }

  T* operator[](std::size_t row) noexcept
  { return &m_data[row * m_ncols]; }
};

int main(int argc, char* argv[])
{
  MPI_Init(&argc, &argv);

  MPI_Comm comm = MPI_COMM_WORLD;
  int rank = MPI_PROC_NULL, size = MPI_PROC_NULL;
  MPI_Comm_rank(comm, &rank);
  MPI_Comm_size(comm, &size);

  const std::size_t maxn = (argc < 2 ? 12 : std::atoi(argv[1]));

  if (maxn % size != 0) {
    std::cerr << "Incorrect size of " << maxn << std::endl;
    MPI_Abort( comm, 1 );
  }

  /* xlocal[][0] is lower ghostpoints, xlocal[][maxn+2] is upper */

  // top and bottom processes have one less row of interior points
  std::size_t i_first = 1;
  std::size_t i_last  = maxn/size;
  if (rank == 0)
    i_first++;
  if (rank == size - 1)
    i_last--;

  // create data
  typedef matrix<double> matrix_type;
  matrix_type xlocal(maxn/size + 2 ,maxn);
  matrix_type xnew(maxn/size + 2, maxn);
  for (std::size_t i=1; i<=maxn/size; i++)
    for (std::size_t j=0; j<maxn; j++)
      xlocal[i][j] = rank;
  for (std::size_t j=0; j<maxn; j++) {
    xlocal[i_first-1][j] = -1;
    xlocal[i_last+1][j] = -1;
  }

  double gdiffnorm  = DBL_MAX;
  int itcnt = 0;

  const double time = MPI_Wtime();
  for (; itcnt<100 && gdiffnorm > 1.0e-3; ++itcnt) {
    /* Send up unless I'm at the top, then receive from below */
    /* Note the use of xlocal[i] for &xlocal[i][0] */
    if (rank < size - 1)
      MPI_Send(xlocal[maxn/size], maxn, MPI_DOUBLE, rank + 1, 0, comm);
    if (rank > 0)
      MPI_Recv(xlocal[0], maxn, MPI_DOUBLE, rank - 1, 0,
               comm, MPI_STATUS_IGNORE);
    /* Send down unless I'm at the bottom */
    if (rank > 0)
      MPI_Send(xlocal[1], maxn, MPI_DOUBLE, rank - 1, 1, comm);
    if (rank < size - 1)
      MPI_Recv(xlocal[maxn/size+1], maxn, MPI_DOUBLE, rank + 1, 1,
               comm, MPI_STATUS_IGNORE);

    /* Compute new values (but not on boundary) */
    double diffnorm = 0.0;
#pragma omp parallel for reduction(+:diffnorm)
    for (std::size_t i=i_first; i<=i_last; i++) {
      for (std::size_t j=1; j<maxn-1; j++) {
        xnew[i][j] = (xlocal[i][j+1] + xlocal[i][j-1] +
                      xlocal[i+1][j] + xlocal[i-1][j]) / 4.0;
        diffnorm += (xnew[i][j] - xlocal[i][j]) * (xnew[i][j] - xlocal[i][j]);
      }
    }

    /* Only transfer the interior points */
#pragma omp parallel for
    for (std::size_t i=i_first; i<=i_last; i++) {
      for (std::size_t j=1; j<maxn-1; j++) {
        xlocal[i][j] = xnew[i][j];
      }
    }

    MPI_Allreduce(&diffnorm, &gdiffnorm, 1, MPI_DOUBLE, MPI_SUM, comm);
    gdiffnorm = std::sqrt(gdiffnorm);
  }
  const double elapsed = MPI_Wtime() - time;

  if (rank==0)
    std::cout << "jacobi_1d_mpi_omp_naive "
              << size                  << ' '
#ifdef _OPENMP
              << omp_get_max_threads() << ' '
#else
              << 1                     << ' '
#endif
              << elapsed               << ' '
              << itcnt                 << "\n";

  MPI_Finalize();
  return 0;
}

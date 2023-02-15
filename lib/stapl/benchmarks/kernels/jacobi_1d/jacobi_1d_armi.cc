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
/// A simple implementation of http://www.mcs.anl.gov/research/projects/mpi/tutorial/mpiexmpl/src/jacobi/C/main.html
//////////////////////////////////////////////////////////////////////

#include <stapl/runtime.hpp>
#include <algorithm>
#include <iostream>
#include <cfloat>
#include <cmath>
#include <vector>

using namespace stapl;

template<typename T>
class matrix
{
private:
  std::size_t    m_nrows;
  std::size_t    m_ncols;
  std::vector<T> m_data;

public:
  matrix(std::size_t nrows, std::size_t ncols)
  : m_nrows(nrows),
    m_ncols(ncols),
    m_data(ncols * nrows)
  { }

  T const& operator()(std::size_t row, std::size_t ncol) const noexcept
  { return m_data[(row * m_ncols) + ncol]; }

  T& operator()(std::size_t row, std::size_t ncol) noexcept
  { return m_data[(row * m_ncols) + ncol]; }

  T const* operator[](std::size_t row) const noexcept
  { return &m_data[row * m_ncols]; }

  T* operator[](std::size_t row) noexcept
  { return &m_data[row * m_ncols]; }
};

template<typename T>
class jacobi_computation
: public p_object
{
private:
  std::size_t m_maxn;
  matrix<T>   xlocal;
  matrix<T>   xnew;
  std::size_t i_first;
  std::size_t i_last;
  double      diffnorm;
  int         rmi_cnt;
  int         cur_rmi_cnt;

private:
  double get_diffnorm(void) const noexcept
  { return diffnorm; }

  template<typename Range>
  void recv_lower(Range const& v) noexcept
  {
    std::copy(std::begin(v), std::end(v), xlocal[0]);
    ++cur_rmi_cnt;
  }

  template<typename Range>
  void recv_upper(Range const& v) noexcept
  {
    std::copy(std::begin(v), std::end(v),
              xlocal[m_maxn/this->get_num_locations()+1]);
    ++cur_rmi_cnt;
  }

  bool all_received(void) noexcept
  {
    if (rmi_cnt!=cur_rmi_cnt)
      return false;
    cur_rmi_cnt = 0;
    return true;
  }

public:
  jacobi_computation(std::size_t n)
  : m_maxn(n),
    xlocal(n/this->get_num_locations() + 2, m_maxn),
    xnew(n/this->get_num_locations() + 2, m_maxn),
    i_first(1),
    i_last(n/this->get_num_locations()),
    diffnorm(0.0),
    rmi_cnt(0),
    cur_rmi_cnt(0)
  {
    // top and bottom locations have one less row of interior points
    if (this->get_location_id() == 0)
      ++i_first;
    if (this->get_location_id() == (this->get_num_locations() - 1))
      --i_last;

    // initialize data
    for (std::size_t i=1; i<=m_maxn/this->get_num_locations(); i++)
      for (std::size_t j=0; j<m_maxn; j++)
        xlocal[i][j] = this->get_location_id();
    for (std::size_t j=0; j<m_maxn; j++) {
      xlocal[i_first-1][j] = -1;
      xlocal[i_last+1][j] = -1;
    }

    // internal location, waits for two requests
    if (this->get_location_id() > 0)
      ++rmi_cnt;
    if (this->get_location_id() < (this->get_num_locations() - 1))
      ++rmi_cnt;

    rmi_fence();
  }

  double iteration(void)
  {
    // send up unless I'm at the top, receive from below
    if (this->get_location_id() < (this->get_num_locations() - 1)) {
      const unsigned int dest = this->get_location_id() + 1;
      auto r = make_range_n(xlocal[m_maxn/this->get_num_locations()], m_maxn);
      async_rmi(dest, this->get_rmi_handle(),
                &jacobi_computation::template recv_upper<decltype(r)>,
                std::move(r));
    }

    // send down unless I'm at the bottom
    if (this->get_location_id() > 0) {
      const unsigned int dest = (this->get_location_id() - 1);
      auto r = make_range_n(xlocal[1], m_maxn);
      async_rmi(dest, this->get_rmi_handle(),
                &jacobi_computation::template recv_lower<decltype(r)>,
                std::move(r));
    }

    block_until([this] { return this->all_received(); });

    // compute new values (but not on boundary)
    diffnorm = 0.0;
    for (std::size_t i=i_first; i<=i_last; i++) {
      for (std::size_t j=1; j<m_maxn-1; j++) {
        xnew[i][j] = (xlocal[i][j+1] + xlocal[i][j-1] +
                      xlocal[i+1][j] + xlocal[i-1][j]) / 4.0;
        diffnorm += (xnew[i][j] - xlocal[i][j]) * (xnew[i][j] - xlocal[i][j]);
      }
    }

    auto f = allreduce_rmi(std::plus<double>{},
                           this->get_rmi_handle(),
                           &jacobi_computation::get_diffnorm);

    for (std::size_t i = i_first; i <= i_last; ++i) {
      for (std::size_t j=1; j<m_maxn-1; ++j) {
        xlocal[i][j] = xnew[i][j];
      }
    }

    return std::sqrt(f.get());
  }
};


exit_code stapl_main(int argc, char* argv[])
{
  const unsigned int rank = get_location_id();
  const unsigned int size = get_num_locations();
  const std::size_t maxn = (argc < 2 ? 12 : std::atoi(argv[1]));
  if (maxn % size != 0) {
    std::cerr << "Incorrect size of " << maxn << std::endl;
    abort("Incorrect size");
  }

  // object for stencil computation
  jacobi_computation<double> m{maxn};
  double gdiffnorm = DBL_MAX;
  int itcnt        = 0;
  counter<default_timer> c;
  c.start();
  for (; itcnt<100 && gdiffnorm > 1.0e-3; ++itcnt) {
    gdiffnorm = m.iteration();
  }
  const double elapsed = c.stop();
  if (rank==0)
    std::cout << "jacobi_1d_armi "
              << get_num_processes() << ' '
              << get_num_locations() / get_num_processes() << ' '
              << elapsed << ' '
              << itcnt << "\n";

  rmi_fence();

  return EXIT_SUCCESS;
}

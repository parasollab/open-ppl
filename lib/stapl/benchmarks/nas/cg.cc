/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

// I don't want libstd or boost trying to use c++0x features for now
// as stapl compile fails...
//
#undef __GXX_EXPERIMENTAL_CXX0X__

#include <cstdio>
#include <cstdlib>
#include <cmath>

#include <stapl/runtime.hpp>

#include <stapl/algorithms/algorithm.hpp>
#include "cg_numeric.hpp"
// #include <test/algorithms/test_utils.h>
#include <views.h>
#include <views/array_2D_view.hpp>
#include <p_array.h>
#include <p_matrix.h>

#include <utility/tuple.h>
#include <iostream>
#include <iomanip>

#include "counting_view.hpp"

#include "matrix.hpp"
#include "matrix_view.hpp"

//#include "matvec.hpp"
#include "vector_operations.hpp"
#include "serial_do_loop.hpp"

using std::size_t;

using namespace stapl;

using stapl::prototype::inner_product;

// base container for p_matrix.
typedef mtl::matrix<
  double, mtl::rectangle<>,
  mtl::compressed<int, mtl::external, mtl::index_from_one>,
  mtl::row_major
>::type b_matrix_type;


typedef my_matrix<b_matrix_type> matrix_type;


extern "C" {
void makea_(int&,int&,double*,int*,int*,int&,
           int&,int&,int&,int&,
           double&,int*,int*,double*,double*,int*,double&);
}


struct problem_traits
{
  int      n;
  size_t   niter;
  int      nonzer;
  double   lambda;
  double   zeta_ref;

  problem_traits()
    : n(0), niter(0), nonzer(0), lambda(0.0l), zeta_ref(0.0l)
  { }

  problem_traits(int _n, size_t _ni, int _nz, double _l, double _z)
    : n(_n), niter(_ni), nonzer(_nz), lambda(_l), zeta_ref(_z)
  { }
};


matrix_type& initialize_matrix(problem_traits& traits,
                               std::size_t nprows,
                               std::size_t npcols,
                               std::size_t my_row,
                               std::size_t my_col)
{
  const int nlocations = get_num_locations();

  // nz is updated by makea to the actual number of nonzeros generated.
  // This is an upper bound.
  //
  int nz =
     traits.n * (traits.nonzer + 1) * (traits.nonzer + 1) / nlocations
           + traits.n * (traits.nonzer + 2 + nlocations / 256) / npcols;

  // FIXME - don't know why, but fortran wants heap alloc'd.
  /*
  //  double   a[nz];
  //  int      colidx[nz];
  //  int      rowstr[traits.n+1];
  //
  //  double   aelt[nz];
  //  double   v[traits.n+1];
  //  int      iv[2*traits.n+1];
  //  int      arow[nz];
  //  int      acol[nz];
  */

  double *a     = new double[nz];
  double *aelt  = new double[nz];
  double *v     = new double[traits.n+1];
  int* colidx   = new int[nz];
  int* rowstr   = new int[traits.n+1];
  int* iv       = new int[2*traits.n+1];
  int* arow     = new int[nz];
  int* acol     = new int[nz];

  // storage for call to makea.
  double   rcond(0.1l);

  // row-wise distribution of A.
  /*
  //
  // const size_t id   = get_location_id();
  // int firstrow      = id * traits.n / nlocations + 1;
  // int lastrow       = firstrow + traits.n / nlocations;
  // int firstcol      = 1;
  // int lastcol       = traits.n;
  */

  const std::size_t col_size = traits.n / npcols;
  const std::size_t row_size = traits.n / nprows;

  int firstcol       = my_col * col_size + 1;
  int lastcol        = firstcol - 1 + col_size;

  int firstrow       = my_row * row_size + 1;
  int lastrow        = firstrow - 1 + row_size;

  makea_(traits.n, nz, a, colidx, rowstr, traits.nonzer,
         firstrow, lastrow, firstcol, lastcol,
         rcond, arow, acol, aelt, v, iv, traits.lambda);

  // FIXME - leaking a, rowstr, and colidx...
  //
  delete[] aelt;
  delete[] v;
  delete[] iv;
  delete[] arow;
  delete[] acol;

  b_matrix_type* base_matrix_ptr =
    new b_matrix_type(traits.n / nlocations, traits.n, nz, a, rowstr, colidx);

  return *(new matrix_type(base_matrix_ptr, nprows, npcols, my_row, my_col));
}


const static problem_traits sample_traits(1400, 15, 7, 10.0l, 8.5971775078648);
const static problem_traits a_traits(14000, 15, 11, 20.0l, 17.130235054029);
const static problem_traits b_traits(75000, 75, 13, 60.0l, 22.712745482631);
const static problem_traits c_traits(150000, 75, 15, 110.0l, 28.973605592845);
const static problem_traits d_traits(1500000, 100, 21, 500.0l, 52.5145321058);
const static problem_traits e_traits(9000000, 100, 26, 1500.0l, 77.522164599383);


void print_header(problem_traits const& traits)
{
  // FIXME - do_onceify
  //
  if (get_location_id() == 0)
  {
    std::cout << "NAS CG Benchmark\n"
              << "Size: " << traits.n << "\n"
              << "Iterations: " << traits.niter << "\n"
              << "Number of active processes: " << get_num_locations() << "\n"
              << "Number of nonzeros per row: " << traits.nonzer << "\n"
              << "Eigenvalue shift: " << traits.lambda << "\n\n"
              << "   iteration           ||r||                 zeta\n";
  }
}


template<typename Reference1, typename Reference2>
void print_iteration(const std::size_t iteration,
                     Reference1 norm_r,
                     Reference2 zeta)
{
  // get_executor()(execute_all);

  // FIXME - do_onceify
  //
  if (get_location_id() == 0)
  {
    std::cout.precision(13);
    std::cout << "    " << std::setw(5) << iteration
              << "       " << std::setw(20) << std::setprecision(14) << norm_r
              << std::setw(20) << std::setprecision(13) << zeta << "\n";
  }
}


template<typename Reference>
void print_results(Reference zeta, double const& ref_zeta)
{
  // get_executor()(execute_all);

  // FIXME - do_onceify
  //
  if (get_location_id() == 0)
  {
    std::cout << "Benchmark completed\n";

    const double error = zeta - ref_zeta;

    if (error <= 1.0e-10)
      std::cout << "VERIFICATION SUCCESSFUL\n";
    else
      std::cout << "VERIFICATION FAILED\n";

    std::cout << "Zeta is"  << std::setw(20) << std::setprecision(12) << zeta  << "\n"
              << "Error is" << std::setw(20) << std::setprecision(12) << error << "\n";
  }
}


void print_time(double const& benchmark_time)
{
  if (get_location_id() == 0)
    std::cout << "Time in seconds = " << benchmark_time << "\n";
}


problem_traits
compute_problem_traits(std::string problem_class)
{
  if (problem_class == "sample")
    return sample_traits;

  if (problem_class == "a")
    return  a_traits;

  if (problem_class == "b")
    return b_traits;

  if (problem_class == "c")
    return c_traits;

  if (problem_class == "d")
    return d_traits;

  abort_rmi("problem class must be a, b, c, d, or sample.\n");

  return problem_traits();
}


template<typename View2D, typename View1D>
struct cg_loop_wf
{
private:
  View2D&  A;
  View1D   x;

public:
  cg_loop_wf(View2D& v1, View1D const& v2)
    : A(v1), x(v2)
  { }

  template<typename IterRef, typename VecView, typename RhoRef>
  auto
  operator()(IterRef n, VecView z, VecView r, VecView p, RhoRef rho) const
    -> decltype(make_tuple(z, r, p, rho))
  {
    auto q       = A * p;

    auto alpha   = rho / inner_product(p, q);

    auto new_z   = z + alpha * p;

    auto new_r   = r - alpha * q;

    auto new_rho = inner_product(new_r, new_r);

    auto beta    = new_rho / rho;

    auto new_p   = new_r + beta * p;

    return make_tuple(new_z, new_r, new_p, new_rho);
  }
}; // struct cg_loop_wf


template<typename View2D, typename View1D>
auto conjugate_gradient(View2D& A, View1D& x)
  -> decltype(make_tuple(euclidean_norm(x), x))
{
  //
  // set initial values of loop variants
  //
  //
  // FIXME - really don't want processor layout detail in at
  // this point / level. Need to expand interface to say fill
  // interface to say use the shape AND distribution of x.
  //   z = vector_fill_n(0.0, x);
  //
  const std::size_t dim = std::sqrt(get_num_locations());
  auto z                = row_vector_fill_n(0.0, x.size(), dim);

  auto r = x;

  auto p = r;

  auto rho = inner_product(r, r);

  cg_loop_wf<View2D, View1D> wf(A, x);

  // run loop and extract z output value
  //
  auto final_z = get<0>(serial_do_loop(25, wf, z, r, p, rho));

  // compute ||r|| = ||x - Az||
  //
  return make_tuple(
     euclidean_norm(x - A * final_z),
     final_z
  );
}


template<typename MatrixView>
struct ip_loop_wf
{
  MatrixView&   A;
  double        lambda;

  ip_loop_wf(MatrixView& a, double const& l)
    : A(a), lambda(l)
  { }

  template<typename IterRef, typename View1D, typename ZetaRef>
  auto operator()(IterRef it, View1D x, ZetaRef) const
    -> decltype(make_tuple(x, lambda + 1.0 / inner_product(x, x)))
  {
    auto cg_ret   = conjugate_gradient(A, x);

    auto norm_r   = get<0>(cg_ret);

    auto z        = get<1>(cg_ret);

    auto zeta     = lambda + 1.0 / inner_product(x, z);

    print_iteration(it, norm_r, zeta);

    auto new_x    = 1.0 / euclidean_norm(z) * z;

    return make_tuple(new_x, zeta);
  }
}; // struct ip_loop_wf


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "Usage: mpiexec -n <#procs> "<< argv[0] << " <problem-class>\n"
              << "Problem classes are sample, a, b, c, or d.\n";
    return EXIT_FAILURE;
  }

  std::size_t dim = std::sqrt(get_num_locations());

  stapl_assert(dim * dim == get_num_locations(), "non square proc count");

  std::size_t nprows  = dim;
  std::size_t npcols  = dim;

  std::size_t my_prow = get_location_id() / npcols;
  std::size_t my_pcol = get_location_id() % npcols;

  problem_traits traits = compute_problem_traits(argv[1]);

  print_header(traits);

  // The matrix A and vector x are allowed to be constructed untimed.
  auto x = row_vector_fill_n(1.0, traits.n, npcols);

  matrix_type& base_container
    = initialize_matrix(traits, nprows, npcols, my_prow, my_pcol);

  typedef my_matrix_view<matrix_type> view_t;

  view_t A(base_container);

  // get_executor()(execute_all);

  rmi_fence();

  default_timer benchmark_timer;

  benchmark_timer.start();

  //
  // Inverse Power Method
  //
  ip_loop_wf<view_t> wf(A, traits.lambda);

  auto zeta = get<1>(serial_do_loop(traits.niter, wf, x, 0.0));

  // get_executor()(execute_all);

  double benchmark_time = benchmark_timer.stop();

  print_results(zeta, traits.zeta_ref);

  print_time(benchmark_time);

  //  delete &base_container;

  return EXIT_SUCCESS;
}

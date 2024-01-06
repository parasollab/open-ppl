/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_NAS_FT_HPP
#define STAPL_BENCHMARKS_NAS_FT_HPP

#include <complex>
#include <cmath>
#include <vector>
#include <algorithm>
#include <array>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/mapping_functions/linearization.hpp>
#include <stapl/skeletons/functional/alltoall.hpp>
#include <stapl/skeletons/functional/map.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/utility/lightweight_vector.hpp>

STAPL_IS_BASIC_TYPE(std::complex<double>)

namespace stapl {

typedef unsigned int            val_t;
typedef unsigned int            int_t;
typedef std::complex<double>    complex_t;


/// To optimize the alltoall operation used in the transpose you can
/// reuse the buffer in the preparation of transpose
// #define USE_BUFFER

#ifdef USE_BUFFER
std::vector<lightweight_vector<complex_t>> buffer;
#endif

/// Defining multiarray views on 1D views results in 15-30% performance
/// degradation in this benchmark.
// #define USE_MULTIARRAY_VIEWS
#ifdef USE_MULTIARRAY_VIEWS
#include <stapl/views/multiarray_view.hpp>
#endif

constexpr double const_pi() { return std::atan(1)*4; }

//////////////////////////////////////////////////////////////////////
/// @brief In this benchmark three types of partitioning can be used.
///
/// @li layout_0d - a 1x1 partitioning, used for the 1 processor case.
/// @li layout_1d - a 1xp partitioning, used for more than 1 processor.
/// @li layout_2d - a p1xp2 partitioning, used for the cases in which
///     nz > p (p1 = nz, p2 = p/nz)
//////////////////////////////////////////////////////////////////////
enum problem_layout {layout_0d, layout_1d, layout_2d};

std::ostream& operator<<(std::ostream& os, problem_layout l)
{
    switch(l) {
         case layout_0d : os << "Layout 0D"; break;
         case layout_1d : os << "Layout 1D"; break;
         case layout_2d : os << "Layout 2D"; break;
         default : os.setstate(std::ios_base::failbit);
    }
    return os;
}


//////////////////////////////////////////////////////////////////////
/// @brief Problem constant traits for the FT benchmark.
//////////////////////////////////////////////////////////////////////
struct problem_traits
{
  int_t nx;
  int_t ny;
  int_t nz;
  int_t niters;
  int_t maxdim;
  char clazz;

  problem_traits(char clazz)
    : clazz(clazz)
  {
    std::tie(nx, ny, nz, niters) = compute_traits(clazz);
    maxdim = std::max({nx, ny, nz});
  }

  tuple<int_t, int_t, int_t, int_t> compute_traits(char clazz)
  {
    switch(clazz) {
      case 'X': // 64x64x64
        return make_tuple(8, 8, 8, 1);
      case 'S': // 64x64x64
        return make_tuple(64, 64, 64, 6);
      case 'W': // 128x128x32
        return make_tuple(128, 128, 32, 6);
      case 'A': // 256x256x128
        return make_tuple(256, 256, 128, 6);
      case 'B': // 512x256x256
        return make_tuple(512, 256, 256, 20);
      case 'C': // 512x512x512
        return make_tuple(512, 512, 512, 20);
      case 'D': // 2048x1024x1024
        return make_tuple(2048, 1024, 1024, 25);
      case 'E': // 4096x2048x2048
        return make_tuple(4096, 2048, 2048, 25);
      default:
        stapl::abort("problem class must be S, W, A, B, C, D, or E.\n");
        return make_tuple(0, 0, 0, 0);
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Holds the configurations for the FT benchmark.
//////////////////////////////////////////////////////////////////////
struct problem_config
{
  typedef std::array<std::size_t, 3> dims_type;
  typedef indexed_domain<std::size_t>                    dim_domain_t;

  int_t me;
  int_t me1;
  int_t me2;
  int_t np;
  int_t np1;
  int_t np2;
  int_t maxdim;
  int_t ntdivp;
  int_t fftblock;
  int_t fftblockpad;
  int_t transblock;
  int_t transblockpad;
  int_t ntotal_f;
  double const alpha;
  double const seed;
  double const a;
  double const pi;
  double const ap;
  int_t nx;
  int_t ny;
  int_t nz;
  problem_layout layout;
  // partition_type partitions;
  std::array<dims_type, 3> dims;
  std::array<std::size_t, 3> xstart, xend, ystart, yend, zstart, zend;

  problem_config(int_t loc_id, int_t num_locs,
                 problem_traits traits)
    : me(loc_id),
      np(num_locs),
      maxdim(traits.maxdim),
      ntdivp(((traits.nx * traits.ny)/np)*traits.nz),
      fftblock(16),
      fftblockpad(18),
      transblock(32),
      transblockpad(34),
      ntotal_f(traits.nx*traits.ny*traits.nz),
      alpha(1.0e-6),
      seed(314159265),
      a(1220703125),
      pi(3.141592653589793238),
      ap(-4.0*alpha*pi*pi),
      nx(traits.nx),
      ny(traits.ny),
      nz(traits.nz)
  {
    if (np == 1) {
      np1 = np2 = 1;
      layout = problem_layout::layout_0d;
    }
    else if (np <= traits.nz) {
      np1 = 1;
      np2 = np;
      layout = problem_layout::layout_1d;
    }
    else {
      np1 = traits.nz;
      np2 = np/traits.nz;
      layout = problem_layout::layout_2d;
    }
    using namespace std;

    stapl::do_once([=]{
      /// NAS Parallel Benchmarks 3.3.1 -- FT Benchmark
      cout << " Size                : " << traits.nx << "x"
                                        << traits.ny << "x"
                                        << traits.nz << endl;
      cout << " Iterations          : " << traits.niters << endl;
      cout << " Number of processes : " << np << endl;
      cout << " Processor array     : " << np1 << "x" << np2 << endl;
      cout << " Layout type         : " << layout << endl;
    });


    // Determine processor coordinates of this processor
    me2 = me % np2;      //! goes from 0...np2-1
    me1 = me/np2;        //! goes from 0...np1-1

    // Find the partitioning information after each transposition.
    switch(layout) {
      case problem_layout::layout_0d:
        dims[0] =
        dims[1] =
        dims[2] = {{traits.nx, traits.ny, traits.nz}};

        for (std::size_t i = 0; i < 3; ++i){
          xstart[i] = ystart[i] = zstart[i] = 0;
          xend[i] = traits.nx - 1;
          yend[i] = traits.ny - 1;
          zend[i] = traits.nz - 1;
        }
        break;
      case problem_layout::layout_1d:
        dims[0] =
        dims[1] = {{traits.nx, traits.ny, traits.nz}};
        // after two transpose across z and y
        dims[2] = {{traits.nz, traits.nx, traits.ny}};
        xstart[0] = 0;
        xend[0] = traits.nx - 1;
        ystart[0] = 0;
        yend[0] = traits.ny - 1;
        zstart[0] = me2 * traits.nz / np2;
        zend[0] = ((me2 + 1) * traits.nz / np2) - 1;

        xstart[1] = 0;
        xend[1] = traits.nx - 1;
        ystart[1] = 0;
        yend[1] = traits.ny - 1;
        zstart[1] = me2 * traits.nz / np2;
        zend[1] = (me2 + 1) * traits.nz / np2 - 1;

        xstart[2] = 0;
        xend[2] = traits.nx - 1;
        ystart[2] = me2 * traits.ny / np2;
        yend[2] = (me2 + 1) * traits.ny / np2 - 1;
        zstart[2] = 0;
        zend[2] = traits.nz - 1;
        break;
      case problem_layout::layout_2d:
        //TODO add start and ends
        dims[0] = {{traits.nx, traits.ny, traits.nz}};
        //after transpose across z
        dims[1] = {{traits.ny, traits.nx, traits.nz}};
        //after transpose across y
        dims[2] = {{traits.nz, traits.nx, traits.ny}};
    }

    for (std::size_t i = 0; i < 3; ++i) {
      dims[i][1] = dims[i][1] / np1;
      dims[i][2] = dims[i][2] / np2;
    }

    if (layout == layout_2d) {
      if (dims[1][0] < fftblock) fftblock = dims[1][0];
      if (dims[1][1] < fftblock) fftblock = dims[1][1];
      if (dims[1][2] < fftblock) fftblock = dims[1][2];
    }
  }

  void define_type(typer& t)
  {
    t.member(me);
    t.member(me1);
    t.member(me2);
    t.member(np);
    t.member(np1);
    t.member(np2);
    t.member(maxdim);
    t.member(ntdivp);
    t.member(fftblock);
    t.member(fftblockpad);
    t.member(transblock);
    t.member(transblockpad);
    t.member(ntotal_f);
    t.member(alpha);
    t.member(seed);
    t.member(a);
    t.member(pi);
    t.member(ap);
    t.member(nx);
    t.member(ny);
    t.member(nz);
    t.member(layout);
    t.member(dims);
    t.member(xstart);
    t.member(xend);
    t.member(ystart);
    t.member(yend);
    t.member(zstart);
    t.member(zend);
  }
};


template <typename T>
struct matrix_2d
{
  T*               m_data;
  std::vector<std::size_t> m_dims;
  typedef tuple<std::size_t, std::size_t> index_type;
  typedef T                               value_type;
  typedef T&                              reference;

  matrix_2d(T* data, std::initializer_list<std::size_t> dims)
    : m_data(data),
      m_dims(dims)
  { }

  reference
  operator[](index_type idx)
  {
    return m_data[std::get<0>(idx)*m_dims[1] + std::get<1>(idx)];
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief The scratch storages used in all iterations of the FFT.
/// @li y0 and y1 - scratch for computations
/// @li u - roots of unity
///
/// @tparam T type of the elements to store.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct fft_info
{
  std::vector<T> y0;
  std::vector<T> y1;
  std::vector<T> u;

  template <typename Config>
  fft_info(std::vector<T> const& ua,
           Config const& config)
    : y0(config.maxdim*config.fftblockpad),
      y1(config.maxdim*config.fftblockpad),
      u(ua)
  { }

  void define_type(typer& t)
  {
    t.member(y0);
    t.member(y1);
    t.member(u);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Computes the modulo of the exponent of a given base @c a
/// (a^exponent) % (2^46).
///
/// Use
/// @li \f$a^n = a^(n/2)*a^(n/2)\f$ if n even
/// @li \f$a^n = a*a^(n-1)\f$       if n odd
///
/// @param a base
/// @return
//////////////////////////////////////////////////////////////////////
double ipow46(double a, double exp_1, double exp_2)
{
  double q, r, n;
  bool two_pow = false;
  if ((exp_2 == 0) or (exp_1 == 0)) {
    r = 1;
  }
  else {
    q = a;
    r = 1;
    n = exp_1;
    two_pow = true;

    while (two_pow) {
      long n2 = n/2;
      if (n2 * 2 == n) {
        randlc(q, q);
        n = n2;
      }
      else {
        n = n * exp_2;
        two_pow = false;
      }
    }

    while (n > 1) {
      long n2 = n/2;
      if (n2 * 2 == n) {
        randlc(q, q);
        n = n2;
      }
      else {
        randlc(r, q);
        n = n - 1;
      }
    }
    randlc(r, q);
  }
  return r;
}

//////////////////////////////////////////////////////////////////////
/// @brief Provides a pointer to the underlying base container. This
/// pointer is needed for the initialization phase of the FT benchmark.
///
/// @param u the view for which the pointer should be provided
///
/// @return a pointer to the base container of @c u
//////////////////////////////////////////////////////////////////////
template <typename U>
auto get_raw_pointer(U&& u) -> decltype(
                                 get_raw_pointer(
                                   std::forward<U>(u),
                                   typename stapl::is_container<
                                     typename
                                       std::decay<U>::type::view_container_type
                                   >::type()))
{
  return get_raw_pointer(std::forward<U>(u),
                         typename stapl::is_container<
                           typename
                             std::decay<U>::type::view_container_type
                         >::type());
}

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for the case that pointer cannot be retrieved.
/// This specialization is needed to avoid compilation errors.
//////////////////////////////////////////////////////////////////////
template <typename U>
void* get_raw_pointer(U&& u, boost::mpl::true_)
{
  stapl_assert(true, "Raw pointer is not accessible on non-native views");
  return nullptr;
}

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for the case that pointer can be retrieved.
//////////////////////////////////////////////////////////////////////
template <typename U>
auto get_raw_pointer(U&& u, boost::mpl::false_)
  -> decltype(u.container().container().data())
{
  return u.container().container().data();
}


//////////////////////////////////////////////////////////////////////
/// @brief Computes the twiddle factors for each combination of i,j,k.
///
/// The twiddle factors in the 3D fft computation are computed as
/// \f$-4\alpha\pi^2|z|^2 \f$ in which
/// \f$|z|^2 = \bar{i}^2+\bar{j}^2+\bar{k}^2\f$. \f$\bar{i}\f$.
///
/// \f$\bar{i}\f$ is defined as:
/// @li \f$\bar{i} = i\ for\ 0 \le i < \frac{n_1}{2}\f$
/// @li \f$\bar{i} = i - n_1\ for\ \frac{n_1}{2} \le i < n_1\f$
///
/// As an example {0, 1, 2, 3, 4, 5, 6, 7} will be converted to
/// {0, 1, 2, 3, -4, -3, -2, -1}
///
/// Therefore, \f$\bar{i} = (i + \frac{n_1}{2})\ \%\ n_1 - \frac{n_1}{2}\f$
///
/// TODO: Following the fortran version this function might differ for each
/// layout. At least the passed in dimensions might be different.
///
/// @param twiddle_factors the 3D input which will hold twiddle factors
/// @param dim             the global dimension of
///
/// @return
//////////////////////////////////////////////////////////////////////
struct compute_indexmap
{
  problem_config m_config;

  typedef void result_type;

  compute_indexmap(problem_config const& config)
    : m_config(config)
  { }

  template <typename TwiddleFactors>
  void operator()(TwiddleFactors&& twid) const
  {
    std::size_t const d0 = m_config.dims[2][0];
    std::size_t const d1 = m_config.dims[2][1];


#ifdef USE_MULTIARRAY_VIEWS
    std::size_t const d2 = m_config.dims[2][2];
    auto twid_view = make_multiarray_view(twid, d2, d1, d0);
    typedef decltype(twid_view.domain().first()) twid_index;
#else
    typedef decltype(twid.domain().first()) idx_t;
    std::size_t const twid_offset = twid.domain().first();
    auto twid_index = [twid_offset, d0, d1]
                      (std::size_t i, std::size_t j, std::size_t k)
                      { return idx_t(twid_offset + i*d1*d0 + j * d0 + k); };
    auto&& twid_view = twid;
#endif

    std::size_t const nx = m_config.nx;
    std::size_t const ny = m_config.ny;
    std::size_t const nz = m_config.nz;
    double const ap = m_config.ap;

    switch(m_config.layout)
    {
      case problem_layout::layout_0d:
        for (std::size_t i = 0; i < m_config.dims[2][0]; ++i)
        {
          int ii  = (i + m_config.xstart[2] + nx/2) % nx - nx/2;
          int ii2 = ii * ii;
          for (std::size_t j = 0; j < m_config.dims[2][1]; ++j) {
            int jj = (j + m_config.ystart[2] + ny/2) % ny - ny/2;
            int ij2 = jj * jj + ii2;
            for (std::size_t k = 0; k < m_config.dims[2][2]; ++k) {
              int kk  = (k + m_config.zstart[2] + nz/2) % nz - nz/2;
              twid_view[twid_index(k, j, i)] =
                std::exp(ap* ((double) (kk*kk + ij2)));
            }
          }
        }
        break;
      case problem_layout::layout_1d:
      case problem_layout::layout_2d:
        for (std::size_t i = 0; i < m_config.dims[2][1]; ++i)
        {
          int ii  = (i + m_config.xstart[2] + nx/2) % nx - nx/2;
          int ii2 = ii * ii;
          for (std::size_t j = 0; j < m_config.dims[2][2]; ++j) {
            int jj = (j + m_config.ystart[2] + ny/2) % ny - ny/2;
            int ij2 = jj * jj + ii2;
            for (std::size_t k = 0; k < m_config.dims[2][0]; ++k) {
              int kk  = (k + m_config.zstart[2] + nz/2) % nz - nz/2;
              twid_view[twid_index(j, i, k)] =
                std::exp(ap* ((double) (kk*kk + ij2)));
            }
          }
        }
        break;
    }
  }

  void define_type(typer& t)
  {
    t.member(m_config);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Computes the initial condition for the FT benchmark. These
/// values are generated with the help of @c randlc and @c vranlc.
///
/// @note This method of initialization can be used for the layout_0d
/// and layout_1d. For the layout_2d, a different initialization process
/// is used and is not included in here
//////////////////////////////////////////////////////////////////////
struct compute_initial_conditions
{
  typedef void result_type;
private:
  problem_config m_config;
public:
  compute_initial_conditions(problem_config const& config)
    : m_config(config)
  { }

  template <typename U0>
  void operator()(U0&& u0) const
  {
    typedef typename U0::value_type::value_type value_t;
    auto u = reinterpret_cast<value_t*>(get_raw_pointer(u0));

    std::size_t const nx = m_config.nx;
    std::size_t const ny = m_config.ny;
    double const a = m_config.a;
    auto const d = m_config.dims[0];

    double start = m_config.seed;
    double an;
    /// Jump to the starting element for our first plane.
    an = ipow46(a, 2*nx, m_config.zstart[0]*ny + m_config.ystart[0]);
    randlc(start, an);
    an = ipow46(a, 2*nx, ny);

    /// Go through by z planes filling in one square at a time.
    for (std::size_t i = 0; i < d[2]; ++i) // nz/np2
    {
      double x0 = start;
      vranlc(2*nx*d[1], x0, a, &u[2*i*d[1]*d[0]]);
      if (i != d[2]-1) {
        randlc(start, an);
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_config);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Prepares the input for the global exchange (@c alltoall)
/// phase of a 3D matrix transpose for square matrices in the FT benchmark
/// and a balanced partitioning is assumed.
///
/// @note This version of 3D transpose can be used on 3D matrices which
/// are stored as 1D arrays.
///
/// @tparam T type of the input values.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct prepare_transpose
{
  typedef std::vector<lightweight_vector<T>>  result_type;
private:
  std::size_t m_n1, m_n2, m_num_chunks;
public:
  prepare_transpose(std::size_t n1, std::size_t n2, std::size_t num_chunks)
    : m_n1(n1),
      m_n2(n2),
      m_num_chunks(num_chunks)
  { }

  template <typename U0, typename U1>
  result_type operator()(U0&& u0, U1&& u1)
  {
#ifdef USE_BUFFER
    buffer.resize(m_num_chunks);
#else
    result_type r;
    r.reserve(m_num_chunks);
#endif
    typedef typename std::decay<U0>::type input_t;
    typedef typename input_t::value_type value_t;
    typedef typename input_t::index_type index_t;

    std::size_t const u0_offset = u0.domain().first();
    std::size_t const u1_offset = u1.domain().first();
    std::size_t const chunk = m_n1*m_n2/m_num_chunks;

    if (m_n1 >= m_n2) {
      for (std::size_t j = 0; j < m_n2; ++j) {
        for (std::size_t  i = 0; i < m_n1; ++i) {
          u1[u1_offset + i * m_n2 + j] = u0[u0_offset + j * m_n1 + i];
        }
      }
    }
    else {
      for (std::size_t i = 0; i < m_n1; ++i) {
        for (std::size_t j = 0; j < m_n2; ++j) {
          u1[u1_offset + i * m_n2 + j] = u0[u0_offset + j * m_n1 + i];
        }
      }
    }

    auto&& it = u1.begin();
    for (std::size_t i = 0; i < m_num_chunks; ++i) {
      auto end_it = it;
      std::advance(end_it, chunk);
#ifdef USE_BUFFER
      buffer[i].resize(chunk);
      std::copy(it, end_it, buffer[i].begin());
#else
      lightweight_vector<T> c;
      c.reserve(chunk);
      for (auto my_it = it; my_it != end_it; ++my_it) {
        c.push_back(*my_it);
      }
      r.push_back(c);
#endif
      it = end_it;
    }

#ifdef USE_BUFFER
    return buffer;
#else
    return r;
#endif
  }

  void define_type(typer& t)
  {
    t.member(m_n1);
    t.member(m_n2);
    t.member(m_num_chunks);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Puts the value received after the global exchange phase of
/// a 3D transpose in the given output.
//////////////////////////////////////////////////////////////////////
struct transpose_finish
{
  typedef bool result_type;

private:
  std::size_t m_n1, m_n2;
public:
  transpose_finish(std::size_t n1, std::size_t n2)
    : m_n1(n1),
      m_n2(n2)
  { }


  template <typename Chunks, typename U0, typename U1>
  result_type operator()(Chunks&& chunks, U0&& u0, U1&& u1)
  {
    typedef typename std::decay<U1>::type::value_type::value_type value_t;

    auto dom = u0.domain();
    auto i   = dom.first();

    for (auto const& chunk : chunks) {
      for (auto&& elem : chunk) {
        u0[i] = elem;
        i = dom.advance(i, 1);
      }
    }

    std::size_t const u0_offset = u0.domain().first();
    std::size_t const u1_offset = u1.domain().first();


    std::size_t const np2 = chunks.size();

    for (std::size_t p = 0; p < np2; ++p) {
      for (std::size_t j = 0; j < m_n1 / np2; j++) {
        for (i = 0; i < m_n2; i++) {
          u1[u1_offset + j * (m_n2 * np2) + p * m_n2 + i] =
            u0[u0_offset + p * m_n2 * (m_n1 / np2) + j * m_n2 + i];
        }
      }
    }
    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_n1);
    t.member(m_n2);
  }

};


//////////////////////////////////////////////////////////////////////
/// @brief Computes the roots of unity to be used in the 3D FFT.
///
/// Roots of unity (a.k.a. nth roots of unity) are the complex numbers
/// which are answers for the equation \f$w^n = 1\f$. These answers form
/// a regular polygon with n sides on the unit circle.
///
/// These answers to this equation are :
/// \f$ \theta = \frac{2\,\pi\,k}{n},\ k \in \{0, ..., n-1\} \f$
///
/// @return a list of n values representing the nth roots of unity.
//////////////////////////////////////////////////////////////////////
template <typename T>
std::vector<T>
fft_init(std::size_t n)
{
  std::vector<T> u(n);

  std::size_t const m = std::log2(n);
  u[0] = m;
  std::size_t ku = 1;
  std::size_t ln = 1;

  for (std::size_t j = 0; j < m; ++j)
  {
    double const t = const_pi() / ln;
    for (std::size_t i = 0; i < ln; ++i) {
      double const ti = i * t;
      u[i + ku] = T(std::cos(ti), std::sin(ti));
    }
    ku += ln;
    ln *= 2;
  }
  return u;
}


//////////////////////////////////////////////////////////////////////
/// @brief Performs the L-th iteration of the second variant of the
/// Stockham FFT.
///
/// @param is  FFT direction
/// @param l   level
/// @param m   first dimension of computation
/// @param n   second dimension of computation
/// @param ny  first dimension of the scratch
/// @param ny1
/// @param u   roots of unity
/// @param x   scratch matrix
/// @param y   scratch matrix
//////////////////////////////////////////////////////////////////////
template <typename X, typename Y, typename U>
void fftz2 (int is, std::size_t l, std::size_t m, std::size_t n,
            std::size_t ny, std::size_t ny1,
            U&& u, X&& x, Y&& y)
{
  /// Set initial parameters.
  typedef typename std::decay<X>::type x_t;
  typedef typename x_t::value_type     value_t;
  typedef typename x_t::index_type     index_t;
  value_t u1;

  std::size_t const n1 = n / 2;
  std::size_t const lk = 1 << (l - 1);
  std::size_t const li = 1 << (m - l);
  std::size_t const lj = 2 * lk;
  std::size_t const ku = li;

  for (std::size_t i = 0; i < li; ++i) {
    std::size_t const i11 = i * lk;
    std::size_t const i12 = i11 + n1;
    std::size_t const i21 = i * lj;
    std::size_t const i22 = i21 + lk;
    value_t const u1 = is ==1 ? u[ku+i] : std::conj(u[ku+i]);

    /// This loop is vectorizable.
    for (std::size_t k = 0; k < lk; ++k) {
      for (std::size_t j = 0; j < ny; ++j) {
        value_t x11, x21;
        x11 = x[index_t(i11+k, j)];
        x21 = x[index_t(i12+k, j)];
        y[index_t(i21+k, j)] = x11 + x21;
        y[index_t(i22+k, j)] = u1 * (x11 - x21);
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////
/// @brief Computes NY N-point complex-to-complex FFTs of X using an
/// algorithm due to Swarztrauber. X is both the input and the output
/// array, while Y is a scratch array.  It is assumed that N = 2^M.
/// Before calling CFFTZ to perform FFTs, the array U must be initialized
/// by calling CFFTZ with IS set to 0 and M set to MX, where MX is the
/// maximum value of M for any subsequent call.
///
/// @param is direction
/// @param m  first dimension of computation
/// @param n  second dimension of computation
/// @param x  scratch matrix
/// @param y  scratch matrix
/// @param u  roots of unity
//////////////////////////////////////////////////////////////////////
template <typename Config, typename Scratch, typename U>
void cfftz (Config config, int is,
            std::size_t m, std::size_t n, Scratch&& x, Scratch&& y,
            U&& u)
{
  typedef typename std::decay<Scratch>::type::index_type index_t;
  std::size_t const mx = (int)(u[0].real());

  if ((is != 1 and is != -1) or
      m < 1 or
      m > mx)
  {
    std::cerr << "CFFTZ: Either U has not been initialized, or else "
              << "one of the input parameters is invalid\n"
              << "is = " << is << ", "
              << "m  = " << m << ", "
              << "mx = " << mx << std::endl;
    exit(1);
  }

  /// Perform one variant of the Stockham FFT.
  for (std::size_t l = 1; l <= m; l += 2) {
    fftz2(is, l, m, n, config.fftblock, config.fftblockpad, u, x, y);
    if (l == m) {
      break;
    }

    fftz2(is, l + 1, m, n, config.fftblock, config.fftblockpad, u, y, x);
  }
  //copy Y to X
  if (m % 2 == 1) {
    for (std::size_t j = 0; j < n; ++j) {
      for (std::size_t i = 0; i < config.fftblock; ++i) {
        x[index_t(j, i)] = y[index_t(j, i)];
      }
    }
  }

}

//////////////////////////////////////////////////////////////////////
/// @brief A wrapper for @c cffts1, @c cffts2, and @c cffts3 operators.
///
/// @tparam Op @c cffts1, @c cffts2, or @c cffts3.
/// @tparam b  drop first argument in execution.
//////////////////////////////////////////////////////////////////////
template <typename Op, bool b>
struct cffts
  : public Op
{
  typedef typename Op::result_type result_type;

  template <typename... Args>
  cffts(Args&&... args)
    : Op(std::forward<Args>(args)...)
  { }


  template <typename... V>
  result_type operator()(V&&... v)
  {
    return apply(std::integral_constant<bool, b>(), std::forward<V>(v)...);
  }

  template <typename... V>
  result_type apply(std::false_type, V&&... v)
  {
    return Op::operator()(std::forward<V>(v)...);
  }

  template <typename A, typename... V>
  result_type apply(std::true_type, A&&, V&&... v)
  {
    return Op::operator()(std::forward<V>(v)...);
  }

  void define_type(typer& t)
  {
    t.base<Op>(*this);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Make a cffts from a given complex FFT algorithm.
///
/// @tparam B    drop first argument in execution.
/// @param cffts @c cffts1, @c cffts2, or @c cffts3.
//////////////////////////////////////////////////////////////////////
template <bool B, typename CFFT>
cffts<CFFT, B>
make_cffts(CFFT&& cfft)
{
  return cffts<CFFT, B>(std::forward<CFFT>(cfft));
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute FFT across the first local dimension.
///
/// @tparam T type of the elements
//////////////////////////////////////////////////////////////////////
template <typename T>
struct cffts1
{
  int m_direction;
  problem_config m_config;
  fft_info<T> m_info;
  std::array<std::size_t, 3> d;

  typedef bool result_type;

  template <typename Dims>
  cffts1(int direction, Dims&& dims,
         fft_info<T> const& info,
         problem_config const& config)
    : m_direction(direction),
      m_config(config),
      m_info(info),
      d(dims)
  { }

  template <typename Input, typename Output>
  result_type operator()(Input&& input, Output&& output)
  {

    typedef typename std::decay<Input>::type input_t;
    typedef typename matrix_2d<T>::index_type s_index_t;

    matrix_2d<T> y0(m_info.y0.data(), {m_config.maxdim, m_config.fftblockpad});
    matrix_2d<T> y1(m_info.y1.data(), {m_config.maxdim, m_config.fftblockpad});

#ifdef USE_MULTIARRAY_VIEWS
    auto input_view = make_multiarray_view(input, d[2], d[1], d[0]);
    auto output_view = make_multiarray_view(output, d[2], d[1], d[0]);
    typedef typename decltype(input_view)::index_type i_idx;
    typedef typename decltype(output_view)::index_type o_idx;
#else
    typedef typename input_t::index_type idx_t;
    idx_t input_offset = input.domain().first();
    idx_t output_offset = output.domain().first();
    auto i_idx = [&](std::size_t z, std::size_t y, std::size_t x)
                 { return idx_t(z*d[1]*d[0] + y*d[0] + x + input_offset); };
    auto o_idx = [&](std::size_t z, std::size_t y, std::size_t x)
                 { return idx_t(z*d[1]*d[0] + y*d[0] + x + output_offset); };
    auto&& input_view = input;
    auto&& output_view = output;
#endif

    for (std::size_t k = 0; k < d[2]; ++k) {
      for (std::size_t jj = 0; jj <= d[1] - m_config.fftblock;
          jj+= m_config.fftblock) {
        /// copy to the scratch
        for (std::size_t j = 0; j < m_config.fftblock; ++j) {
          std::size_t const i_j = j+jj;
          for (std::size_t i = 0; i < d[0]; ++i) {
            y0[s_index_t(i, j)] =  input_view[i_idx(k,i_j,i)];
          }
        }

        cfftz(m_config, m_direction, std::log2(d[0]), d[0], y0, y1, m_info.u);

        /// copy back from scratch to the output
        for (std::size_t j = 0; j < m_config.fftblock; ++j) {
          std::size_t const i_j = j+jj;
          for (std::size_t i = 0; i < d[0]; ++i) {
            output_view[o_idx(k,i_j,i)] = y0[s_index_t(i,j)];
          }
        }
      }
    }

    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_direction);
    t.member(m_config);
    t.member(m_info);
    t.member(d);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compute FFT across the second local dimension.
///
/// @tparam T type of the elements
//////////////////////////////////////////////////////////////////////
template <typename T>
struct cffts2
{
  int m_direction;
  problem_config m_config;
  fft_info<T> m_info;
  std::array<std::size_t, 3> d;

  typedef bool result_type;

  template <typename Dims>
  cffts2(int direction, Dims&& dims,
         fft_info<T> const& info,
         problem_config const& config)
    : m_direction(direction),
      m_config(config),
      m_info(info),
      d(dims)
  { }

  template <typename Input, typename Output>
  result_type operator()(Input&& input, Output&& output)
  {
    typedef typename std::decay<Input>::type  input_t;
    typedef typename matrix_2d<T>::index_type s_index_t;

    matrix_2d<T> y0(m_info.y0.data(), {m_config.maxdim, m_config.fftblockpad});
    matrix_2d<T> y1(m_info.y1.data(), {m_config.maxdim, m_config.fftblockpad});

#ifdef USE_MULTIARRAY_VIEWS
    auto input_view = make_multiarray_view(input, d[2], d[1], d[0]);
    auto output_view = make_multiarray_view(output, d[2], d[1], d[0]);
    typedef typename decltype(input_view)::index_type i_idx;
    typedef typename decltype(output_view)::index_type o_idx;
#else
    typedef decltype(input.domain().first()) idx_t;
    idx_t input_offset = input.domain().first();
    idx_t output_offset = output.domain().first();
    auto i_idx = [&](std::size_t z, std::size_t y, std::size_t x)
                 { return idx_t(z*d[1]*d[0] + y*d[0] + x + input_offset); };
    auto o_idx = [&](std::size_t z, std::size_t y, std::size_t x)
                 { return idx_t(z*d[1]*d[0] + y*d[0] + x + output_offset); };
    auto&& input_view = input;
    auto&& output_view = output;
#endif

    for (std::size_t k = 0; k < d[2]; ++k) {
      for (std::size_t ii = 0; ii <= d[0] - m_config.fftblock;
          ii+= m_config.fftblock) {
        /// copy to scratch
        for (std::size_t j = 0; j < d[1]; ++j) {
          for (std::size_t i = 0; i < m_config.fftblock; ++i) {
            std::size_t const i_i = i + ii;
            y0[s_index_t(j, i)] = input_view[i_idx(k,j,i_i)];
          }
        }

        cfftz(m_config, m_direction, std::log2(d[1]), d[1], y0, y1, m_info.u);

        /// copy back from scratch to the output
        for (std::size_t j = 0; j < d[1]; ++j) {
          for (std::size_t i = 0; i < m_config.fftblock; ++i) {
            std::size_t const i_i = i+ii;
            output_view[o_idx(k,j,i_i)] = y0[s_index_t(j, i)];
          }
        }
      }
    }
    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_direction);
    t.member(m_config);
    t.member(m_info);
    t.member(d);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compute FFT across the third local dimension.
///
/// @tparam T type of the elements
//////////////////////////////////////////////////////////////////////
template <typename T>
struct cffts3
{
  int m_direction;
  problem_config m_config;
  fft_info<T> m_info;
  std::array<std::size_t, 3> d;

  typedef bool result_type;

  template <typename Dims>
  cffts3(int direction, Dims&& dims,
         fft_info<T> const& info,
         problem_config const& config)
    : m_direction(direction),
      m_config(config),
      m_info(info),
      d(dims)
  { }


  template <typename Input, typename Output>
  result_type operator()(Input&& input, Output&& output)
  {

    typedef typename std::decay<Input>::type  input_t;
    typedef typename matrix_2d<T>::index_type s_index_t;

    matrix_2d<T> y0(m_info.y0.data(), {m_config.maxdim, m_config.fftblockpad});
    matrix_2d<T> y1(m_info.y1.data(), {m_config.maxdim, m_config.fftblockpad});

#ifdef USE_MULTIARRAY_VIEWS
    auto input_view = make_multiarray_view(input, d[2], d[1], d[0]);
    auto output_view = make_multiarray_view(output, d[2], d[1], d[0]);
    typedef typename decltype(input_view)::index_type i_idx;
    typedef typename decltype(output_view)::index_type o_idx;
#else
    typedef decltype(input.domain().first()) idx_t;
    idx_t input_offset = input.domain().first();
    idx_t output_offset = output.domain().first();
    auto i_idx = [&](std::size_t z, std::size_t y, std::size_t x)
                 { return idx_t(z*d[1]*d[0] + y*d[0] + x + input_offset); };
    auto o_idx = [&](std::size_t z, std::size_t y, std::size_t x)
                 { return idx_t(z*d[1]*d[0] + y*d[0] + x + output_offset); };
    auto&& input_view = input;
    auto&& output_view = output;
#endif

    for (std::size_t j = 0; j < d[1]; ++j) {
      for (std::size_t ii = 0; ii <= d[0] - m_config.fftblock;
          ii+= m_config.fftblock) {
        /// copy to scratch
        for (std::size_t k = 0; k < d[2]; ++k) {
          for (std::size_t i = 0; i < m_config.fftblock; ++i) {
            std::size_t const i_i = i + ii;
            y0[s_index_t(k, i)] = input_view[i_idx(k,j,i_i)];
          }
        }

        cfftz(m_config, m_direction, std::log2(d[2]), d[2], y0, y1, m_info.u);

        /// copy back from scratch to the output
        for (std::size_t k = 0; k < d[2]; ++k) {
          for (std::size_t i = 0; i < m_config.fftblock; ++i) {
            std::size_t const i_i = i+ii;
            output_view[o_idx(k,j,i_i)] = y0[s_index_t(k, i)];
          }
        }
      }
    }
    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_direction);
    t.member(m_config);
    t.member(m_info);
    t.member(d);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Evolve the input by multiplying by pairwise multiplication
/// with the twiddle factors.
//////////////////////////////////////////////////////////////////////
struct evolve
{
  typedef void result_type;

  std::array<std::size_t, 3> d;

  template <typename Dims>
  evolve(Dims&& dims)
    : d(dims)
  { }

  template <typename U0, typename U1, typename TwiddleFactors>
  result_type operator()(U0&& u0, U1&& u1, TwiddleFactors&& twid)
  {
    typedef typename std::decay<U0>::type    input_t;
    typedef typename input_t::value_type     value_t;

#ifdef USE_MULTIARRAY_VIEWS
    auto twid_view = make_multiarray_view(twid, d[2], d[1], d[0]);
    auto u0_view = make_multiarray_view(u0, d[2], d[1], d[0]);
    auto u1_view = make_multiarray_view(u1, d[2], d[1], d[0]);
    typedef decltype(twid_view.domain().first()) twid_idx;
    typedef decltype(u0_view.domain().first())   u0_idx;
    typedef decltype(u1_view.domain().first())   u1_idx;
#else
    typedef decltype(u0.domain().first()) idx_t;
    std::size_t const twid_offset = twid.domain().first();
    std::size_t const u0_offset = u0.domain().first();
    std::size_t const u1_offset = u1.domain().first();

    std::array<std::size_t, 3> di = d;
    auto twid_idx = [&di, twid_offset]
                    (std::size_t z, std::size_t y, std::size_t x)
                    {return idx_t(z*di[1]*di[0] + y*di[0] + x + twid_offset);};
    auto u0_idx = [&di, u0_offset]
                  (std::size_t z, std::size_t y, std::size_t x)
                  {return idx_t(z*di[1]*di[0] + y*di[0] + x + u0_offset);};
    auto u1_idx = [&di, u1_offset]
                  (std::size_t z, std::size_t y, std::size_t x)
                  {return idx_t(z*di[1]*di[0] + y*di[0] + x + u1_offset);};

    auto&& twid_view = twid;
    auto&& u0_view = u0;
    auto&& u1_view = u1;
#endif


    for (std::size_t k = 0; k < d[2]; k++){
      for (std::size_t j = 0; j < d[1]; j++){
        for (std::size_t i = 0; i < d[0]; i++){
          value_t v(twid_view[twid_idx(k, j, i)]);
          u0_view[u0_idx(k,j,i)] = u0_view[u0_idx(k,j,i)] * v;
          u1_view[u1_idx(k,j,i)] = u0_view[u0_idx(k,j,i)];
        }
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(d);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Compute the checksum for the FT benchmark computations
/// using a predefined selection of the computed values.
///
/// @tparam T type of the elements.
//////////////////////////////////////////////////////////////////////
template <typename T>
struct checksum
{
  typedef T result_type;

  problem_config m_config;
  std::array<std::size_t, 3> d;

  template <typename Dims>
  checksum(problem_config const& config,
           Dims&& dims)
    : m_config(config),
      d(dims)
  { }

  template <typename U2>
  result_type operator()(U2&& u2)
  {
    typedef typename std::decay<U2>::type input_t;
    typedef typename input_t::value_type  value_t;

    value_t chk(0.,0.);

#ifdef USE_MULTIARRAY_VIEWS
    auto u2_view = make_multiarray_view(u2, d[2], d[1], d[0]);
    typedef decltype(u2_view.domain().first()) u2_index;
#else
    typedef decltype(u2.domain().first()) idx_t;
    std::size_t u2_offset = u2.domain().first();
    auto u2_index = [&](std::size_t z, std::size_t y, std::size_t x)
                    { return idx_t(z*d[1]*d[0] + y*d[0] + x + u2_offset); };
    auto&& u2_view = u2;
#endif

    std::size_t nx = m_config.nx;
    std::size_t ny = m_config.ny;
    std::size_t nz = m_config.nz;

    /* Work on the local data,  */
    //We should use the given dimensions
    for (std::size_t j = 1; j <= 1024; j++) {
      std::size_t q = (j % nx) + 1;
      if (q >= (m_config.xstart[0]+1) && q <= (m_config.xend[0]+1)) {
        std::size_t r = ((3 * j) % ny) + 1;
        if (r >= (m_config.ystart[0]+1) && r <= (m_config.yend[0]+1)) {
          std::size_t s = ((5 * j) % nz) + 1;
          if (s >= (m_config.zstart[0]+1) && s <= (m_config.zend[0]+1)) {
            std::size_t ox, oy, oz;
            oz = s-(m_config.zstart[0]+1);
            oy = r-(m_config.ystart[0]+1);
            ox = q-(m_config.xstart[0]+1);
            chk = chk + u2_view[u2_index(oz, oy, ox)];
          }
        }
      }
    }
    chk /= m_config.ntotal_f;
    return chk;
  }

  void define_type(typer& t)
  {
    t.member(m_config);
    t.member(d);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief The FT benchmark uses a predefined distribution for the
/// generated numbers. This distribution simplifies the verification
/// phase by providing a set of numbers to be used to verify the checksums
/// of each iteration of the benchmark.
///
/// @tparam T      type of the elements
/// @param  traits to determine the class of the application
///
/// @return true if verification succeeds.
//////////////////////////////////////////////////////////////////////
template <typename T, typename Traits>
bool verify(Traits traits, T& sums){
  typedef T vec_t;
  vec_t vdata;
  switch (traits.clazz)
  {
    case 'S':
      vdata = vec_t({{0.0, 0.0},
                     {5.546087004964e+02, 4.845363331978e+02},
                     {5.546385409189e+02, 4.865304269511e+02},
                     {5.546148406171e+02, 4.883910722336e+02},
                     {5.545423607415e+02, 4.901273169046e+02},
                     {5.544255039624e+02, 4.917475857993e+02},
                     {5.542683411902e+02, 4.932597244941e+02}});

      break;
    case 'W':
      vdata = vec_t({{0.0, 0.0},
                     {5.673612178944e+02, 5.293246849175e+02},
                     {5.631436885271e+02, 5.282149986629e+02},
                     {5.594024089970e+02, 5.270996558037e+02},
                     {5.560698047020e+02, 5.260027904925e+02},
                     {5.530898991250e+02, 5.249400845633e+02},
                     {5.504159734538e+02, 5.239212247086e+02}});
      break;
    case 'A':
      vdata = vec_t({{0.0, 0.0},
                     {5.046735008193e+02, 5.114047905510e+02},
                     {5.059412319734e+02, 5.098809666433e+02},
                     {5.069376896287e+02, 5.098144042213e+02},
                     {5.077892868474e+02, 5.101336130759e+02},
                     {5.085233095391e+02, 5.104914655194e+02},
                     {5.091487099959e+02, 5.107917842803e+02}});
      break;
    case 'B':
      vdata = vec_t({{0.0, 0.0},
                     {5.177643571579e+02, 5.077803458597e+02},
                     {5.154521291263e+02, 5.088249431599e+02},
                     {5.146409228649e+02, 5.096208912659e+02},
                     {5.142378756213e+02, 5.101023387619e+02},
                     {5.139626667737e+02, 5.103976610617e+02},
                     {5.137423460082e+02, 5.105948019802e+02},
                     {5.135547056878e+02, 5.107404165783e+02},
                     {5.133910925466e+02, 5.108576573661e+02},
                     {5.132470705390e+02, 5.109577278523e+02},
                     {5.131197729984e+02, 5.110460304483e+02},
                     {5.130070319283e+02, 5.111252433800e+02},
                     {5.129070537032e+02, 5.111968077718e+02},
                     {5.128182883502e+02, 5.112616233064e+02},
                     {5.127393733383e+02, 5.113203605551e+02},
                     {5.126691062020e+02, 5.113735928093e+02},
                     {5.126064276004e+02, 5.114218460548e+02},
                     {5.125504076570e+02, 5.114656139760e+02},
                     {5.125002331720e+02, 5.115053595966e+02},
                     {5.124551951846e+02, 5.115415130407e+02},
                     {5.124146770029e+02, 5.115744692211e+02}});
      break;
    case 'C':
      vdata = vec_t({{0.0, 0.0},
                     {5.195078707457e+02, 5.149019699238e+02},
                     {5.155422171134e+02, 5.127578201997e+02},
                     {5.144678022222e+02, 5.122251847514e+02},
                     {5.140150594328e+02, 5.121090289018e+02},
                     {5.137550426810e+02, 5.121143685824e+02},
                     {5.135811056728e+02, 5.121496764568e+02},
                     {5.134569343165e+02, 5.121870921893e+02},
                     {5.133651975661e+02, 5.122193250322e+02},
                     {5.132955192805e+02, 5.122454735794e+02},
                     {5.132410471738e+02, 5.122663649603e+02},
                     {5.131971141679e+02, 5.122830879827e+02},
                     {5.131605205716e+02, 5.122965869718e+02},
                     {5.131290734194e+02, 5.123075927445e+02},
                     {5.131012720314e+02, 5.123166486553e+02},
                     {5.130760908195e+02, 5.123241541685e+02},
                     {5.130528295923e+02, 5.123304037599e+02},
                     {5.130310107773e+02, 5.123356167976e+02},
                     {5.130103090133e+02, 5.123399592211e+02},
                     {5.129905029333e+02, 5.123435588985e+02},
                     {5.129714421109e+02, 5.123465164008e+02}});

      break;
    case 'D':
      vdata = vec_t({{0.0, 0.0},
                     {5.122230065252e+02, 5.118534037109e+02},
                     {5.120463975765e+02, 5.117061181082e+02},
                     {5.119865766760e+02, 5.117096364601e+02},
                     {5.119518799488e+02, 5.117373863950e+02},
                     {5.119269088223e+02, 5.117680347632e+02},
                     {5.119082416858e+02, 5.117967875532e+02},
                     {5.118943814638e+02, 5.118225281841e+02},
                     {5.118842385057e+02, 5.118451629348e+02},
                     {5.118769435632e+02, 5.118649119387e+02},
                     {5.118718203448e+02, 5.118820803844e+02},
                     {5.118683569061e+02, 5.118969781011e+02},
                     {5.118661708593e+02, 5.119098918835e+02},
                     {5.118649768950e+02, 5.119210777066e+02},
                     {5.118645605626e+02, 5.119307604484e+02},
                     {5.118647586618e+02, 5.119391362671e+02},
                     {5.118654451572e+02, 5.119463757241e+02},
                     {5.118665212451e+02, 5.119526269238e+02},
                     {5.118679083821e+02, 5.119580184108e+02},
                     {5.118695433664e+02, 5.119626617538e+02},
                     {5.118713748264e+02, 5.119666538138e+02},
                     {5.118733606701e+02, 5.119700787219e+02},
                     {5.118754661974e+02, 5.119730095953e+02},
                     {5.118776626738e+02, 5.119755100241e+02},
                     {5.118799262314e+02, 5.119776353561e+02},
                     {5.118822370068e+02, 5.119794338060e+02}});
  }

  double epsilon = 1.0e-12;
  bool verified = true;

  for (std::size_t i = 1; i <= traits.niters; i++) {
      if (fabs((sums[i] - vdata[i]) / vdata[i]) > epsilon) {
        verified = false;
        break;
      }
  }
  return verified;
}

} // namespace stapl

#endif // STAPL_BENCHMARKS_NAS_FT_HPP

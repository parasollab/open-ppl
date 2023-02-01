/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <iostream>
#include <iomanip>
#include <array>

#include <stapl/algorithms/algorithm.hpp>
#include "nas_rand.hpp"
#include <stapl/views/counting_view.hpp>

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/skeletons/utility/tags.hpp>
using std::size_t;

struct problem_traits
{
  double   a;
  double   s;
  double   epsilon;

  int      m;
  double   sum_x;
  double   sum_y;

  problem_traits(int m1, double const& sx, double const& sy)
    : a(1220703125.0), s(271828183.0), epsilon(1.0e-8l),
      m(m1), sum_x(sx), sum_y(sy)
  { }
};


const static problem_traits s1_traits(24, -3.247834652034740e+03l,
                                          -6.958407078382297e+03l);
const static problem_traits s2_traits(25, -2.863319731645753e+03l,
                                          -6.320053679109499e+03l);
const static problem_traits a_traits (28, -4.295875165629892e+03l,
                                          -1.580732573678431e+04l);
const static problem_traits b_traits (30,  4.033815542441498e+04l,
                                          -2.660669192809235e+04l);
const static problem_traits c_traits (32,  4.764367927995374e+04l,
                                          -8.084072988043731e+04l);
const static problem_traits d_traits (36,  1.982481200946593e+05l,
                                          -1.020596636361769e+05l);


struct deviate_info
{
private:
  double       m_sum_x;
  double       m_sum_y;
  std::size_t  m_annulus_cnts[10];

public:
  deviate_info()
  { }

  deviate_info(double const& sum_x, double const& sum_y,
               const std::size_t cnts[10])
    : m_sum_x(sum_x), m_sum_y(sum_y)
  {
    for (int i=0; i < 10; ++i)
      m_annulus_cnts[i] = cnts[i];
  }

  void define_type(stapl::typer &t)
  {
    t.member(m_sum_x);
    t.member(m_sum_y);
    t.member(m_annulus_cnts);
  }

  double const& sum_x() const { return m_sum_x; }
  double const& sum_y() const { return m_sum_y; }

  std::size_t const*
  annulus_counts() const
  {
    return m_annulus_cnts;
  }

  deviate_info operator+(deviate_info const& rhs) const
  {
    std::size_t cnt[10];

    std::transform(this->annulus_counts(), this->annulus_counts() + 10,
                   rhs.annulus_counts(), cnt, std::plus<size_t>());

    return deviate_info(m_sum_x + rhs.m_sum_x, m_sum_y + rhs.m_sum_y, cnt);
  }
}; // struct deviate_info


struct initialize_wf
{
  typedef void result_type;

  double m_a;
  double m_s;

  initialize_wf(double const& a, double const& s)
    : m_a(a), m_s(s)
  { }

  void define_type(stapl::typer& t)
  {
    t.member(m_a);
    t.member(m_s);
  }

  template<typename View>
  struct get_address
  {
    typename View::value_type*
    operator()(View& vw) const
    {
      stapl_assert(0, "get_address called on non-fast view");
      return NULL;
    }
  };

  template<typename View>
  struct get_address<stapl::localized_view<View> >
  {
    typedef stapl::localized_view<View> view_t;

    typename view_t::value_type*
    operator()(view_t& vw) const
    {
      return stapl::proxy_core_access::accessor(*vw.begin()).t;
    }
  };

  template <typename View>
  void operator()(View& vw) const
  {
    const size_t offset = vw.domain().first();

    const double seed = compute_seed(m_s, m_a, offset);

    vranlc(vw.size() * 2, seed, m_a, &get_address<View>()(vw)->first);
  }
};


inline
void attempt_deviate(double const& x, double const& y,
                     double& sum_x, double& sum_y, size_t cnts[10])
{
  const double t1 = x*x + y*y;

  if (t1 <= 1.0l)
  {
    const double t2 = std::sqrt(-2.0l * std::log(t1) / t1);
    const double t3 = x * t2;
    const double t4 = y * t2;
    const size_t l  = std::max(std::abs(t3), std::abs(t4));

    ++cnts[l];
    sum_x += t3;
    sum_y += t4;
  }
}


struct identify_deviate
{
  typedef deviate_info result_type;

  template <typename Point>
  result_type
  operator()(Point const& p) const
  {
    // FIXME - avoid initialization, then assignment with some proxying...
    //
    // FIXME - rework without pair. (by two view)...
    ///Data gp(p);

    ///gp.first  = 2.0l * gp.first - 1.0l;
    ///gp.second = 2.0l * gp.second - 1.0l;
    ///
    const std::size_t cnts[10] = {0,0,0,0,0,0,0,0,0,0};

    const double sum_x = 0;
    const double sum_y = 0;

    ///attempt_deviate(gp.first, gp.second, sum_x, sum_y, cnts);

    return deviate_info(sum_x, sum_y, cnts);
  }
};


struct fine_ep_wf
{
  typedef deviate_info result_type;

  double m_a;
  double m_s;

  fine_ep_wf(double const& a, double const& s)
    : m_a(a), m_s(s)
  { }

  void define_type(stapl::typer& t)
  {
    t.member(m_a);
    t.member(m_s);
  }

  template <typename Reference>
  result_type
  operator()(Reference /* const& */ ref) const
  {
    std::size_t cnts[10] = {0,0,0,0,0,0,0,0,0,0};

    double sum_x = 0;
    double sum_y = 0;

    const size_t offset = ref;
    double seed = compute_seed(m_s, m_a, offset);

    const double x = 2.0l * randlc(seed, m_a) - 1.0l;
    const double y = 2.0l * randlc(seed, m_a) - 1.0l;

    attempt_deviate(x, y, sum_x, sum_y, cnts);

    return deviate_info(sum_x, sum_y, cnts);
  }
}; // fine_identify_deviate


struct coarse_ep_wf
{
  typedef deviate_info result_type;

  double const m_a;
  double const m_s;

  coarse_ep_wf(double const& a, double const& s)
    : m_a(a), m_s(s)
  { }

  void define_type(stapl::typer& t)
  {
    t.member(m_a);
    t.member(m_s);
  }

  template <typename View>
  result_type
  operator()(View const& vw) const
  {
    std::size_t cnts[10] = {0,0,0,0,0,0,0,0,0,0};
    double      sum_x = 0.;
    double      sum_y = 0.;

    constexpr unsigned int mk = 16;

    const size_t offset = *vw.begin() >> mk;
    std::size_t num_buckets = vw.size() >> mk;

    std::array<double, 1 << (mk + 1)> X;
    double t1 = 0., t2 = 0.;
    constexpr std::size_t num_elems = 1 << mk;

    t1 = m_a;
    vranlc(0, t1, m_a, X.data());
    t1 = m_a;

    for (unsigned int r = 0 ; r <= mk; ++r) {
      t2 = randlc(t1, t1);
    }

    double an = t1;
    for (std::size_t i = 0; i < num_buckets; ++i) {

      std::size_t kk = offset + i;
      t1 = m_s;
      t2 = an;
      for (unsigned int r = 0 ; r < 100; ++r) {
        std::size_t ik = kk / 2;
        if (2 * ik != kk) {
          randlc(t1, t2);
        }
        if (ik == 0) {
          break;
        }
        randlc(t2, t2);
        kk = ik;
      }

      vranlc(num_elems*2, t1, m_a, X.data());

      for (unsigned int j = 0; j < num_elems; ++j) {
        const double x1 = 2.0l * X[2*j] - 1.0l;
        const double x2 = 2.0l * X[2*j+1] - 1.0l;

        t1 = x1 * x1 + x2 * x2;

        if (t1 <= 1.0l)
        {
          t2 = std::sqrt(-2.0l * std::log(t1) / t1);
          const double t3 = x1 * t2;
          const double t4 = x2 * t2;
          const size_t l  = std::max(std::abs(t3), std::abs(t4));

          ++cnts[l];
          sum_x += t3;
          sum_y += t4;
        }
      }
    }
    return deviate_info(sum_x, sum_y, cnts);
  }
};


problem_traits
compute_problem_traits(std::string const& problem_class)
{
  if (problem_class == "s1")
    return s1_traits;

  if (problem_class == "s2")
    return s2_traits;

  if (problem_class == "a")
    return  a_traits;

  if (problem_class == "b")
    return b_traits;

  if (problem_class == "c")
    return c_traits;

  if (problem_class == "d")
    return d_traits;

  stapl::abort("problem class must be a, b, c, d, s1, or s2.\n");
  return d_traits;
}


struct timer_ident_wf
{
  double m_time;

  typedef double result_type;

  timer_ident_wf(double const& time)
    : m_time(time)
  { }

  void define_type(stapl::typer &t)
  {
    stapl_assert(false, "Should not be packing timer_ident_wf");
  }

  template<typename View>
  double operator()(View const&) const
  {
    return m_time;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using std::cout;
  using std::setw;
  using std::setprecision;
  using stapl::is_coarse_wf;

  if (argc < 2) {
    std::cerr << "Usage: mpiexec -n <#procs> "<< argv[0] << " <problem-class>\n"
              << "Problem classes are a, b, c, d, s1 or s2.\n";
    return EXIT_FAILURE;
  }

  const problem_traits traits = compute_problem_traits(argv[1]);

  const size_t n = std::pow(2.0, traits.m);

  if (stapl::get_location_id() == 0)
  {
    cout << "NAS EP Benchmark\n"
         << "Size: " << n << "\n"
         << "Number of active processes: "
         << stapl::get_num_locations() << "\n\n";
  }

  //
  // Start the Timer
  //
  stapl::counter<stapl::default_timer> tmr;

  // barrier to get all the timers in sync.
  stapl::rmi_fence();

  tmr.start();

  //
  // Generate Points, Identify deviates, sum x & y, reduce annulus counts
  //
  // fine_ep_wf   fmr_wf(traits.a, traits.s);
  //
  // deviate_info res = map_reduce(fmr_wf, sum_deviates(), counting_view(n));

  coarse_ep_wf              cmr_wf(traits.a, traits.s);
  stapl::plus<deviate_info> red_wf;

  typedef std::size_t value_t;
  const deviate_info res =
    stapl::map_reduce<
      stapl::skeletons::tags::with_coarsened_wf
    >(cmr_wf, red_wf, stapl::counting_view<value_t>(n, value_t(0)));

  size_t gc = 0;

  if (stapl::get_location_id() == 0)
  {
    std::cout << "Counts\n";

    for (int i = 0; i < 10; ++i)
    {
      std::cout << i << ": " << res.annulus_counts()[i] << "\n";
      gc += res.annulus_counts()[i];
    }
  }

  const double time = tmr.stop();

  const double global_time =
    map_reduce(timer_ident_wf(time), stapl::max<double>(),
               stapl::counting_view<value_t>(
                 stapl::get_num_locations(), value_t(0)));

  if (stapl::get_location_id() == 0)
  {
    cout << "Benchmark completed\n";

    const double error_x = std::abs((res.sum_x()-traits.sum_x) / res.sum_x());
    const double error_y = std::abs((res.sum_y()-traits.sum_y) / res.sum_y());

    if ((error_x <= traits.epsilon) && (error_y <= traits.epsilon))
      cout << "VERIFICATION SUCCESSFUL\n";
    else
      cout << "VERIFICATION FAILED\n"
           << "epsilon = " << traits.epsilon << "\n"
           << "sx err = "  << error_x << "\n"
           << "sy err = "  << error_y << "\n";

    cout << "EP Benchmark Results:\n"
         << "CPU Time " << setw(10) << setprecision(4) << global_time << "\n"
         << "N = " << n << "\n"
         << " No. Gaussian Pairs = " << gc << "\n"
         << "Sums = " << setw(20) << setprecision(16) << res.sum_x() << " "
         << res.sum_y() << "\n"
         << "Counts:\n";
  }

  return EXIT_SUCCESS;
}

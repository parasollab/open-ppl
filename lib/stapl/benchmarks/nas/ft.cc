/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <cstdlib>
#include <iomanip>
#include <algorithm>
#include <array>
#include <complex>
#include <stapl/runtime.hpp>
#include <stapl/utility/do_once.hpp>
#include <stapl/array.hpp>
#include <stapl/skeletons/map.hpp>
#include <stapl/skeletons/map_reduce.hpp>
#include <stapl/skeletons/executors/execute.hpp>


#include "nas_rand.hpp"
#include "ft.hpp"

using namespace stapl;

template <typename T>
struct fft
{
  bool                m_is_forward;
  std::vector<T>      m_roots_of_unity;
  problem_config      m_config;

  typedef void result_type;

  fft(int is_forward, std::vector<T> const& roots_of_unity,
             problem_config const& config)
    : m_is_forward(is_forward == 1),
      m_roots_of_unity(roots_of_unity),
      m_config(config)
  { }

  template <typename X1, typename X2>
  void operator()(X1&& x1, X2&& x2)
  {
    DECLARE_INLINE_PLACEHOLDERS(4, v);
    DECLARE_INLINE_INPUT_PLACEHOLDERS(3, input);
    typedef typename std::decay<X1>::type::value_type val_t;

    fft_info<val_t> info(m_roots_of_unity, m_config);

    using namespace skeletons;
    using skeletons::tags::with_coarsened_wf;
    using skeletons::map;
    if (m_config.layout == layout_0d) {
      if (m_is_forward)
      {
        auto ft_skeleton =
          compose<tags::inline_flow>(
            v0 << zip<2>(make_cffts<false>(cffts1<val_t>(1, m_config.dims[0],
                                          info, m_config))) | (input0, input0),
            v1 << zip<3>(make_cffts<true>(cffts2<val_t>(1, m_config.dims[1],
                                         info, m_config))) | (v0,input0,input0),
            v2 << zip<3>(make_cffts<true>(cffts3<val_t>(1, m_config.dims[2],
                                          info, m_config))) | (v1,input0,input1)
          );
        skeletons::execute(
          skeletons::execution_params(default_coarsener()),
          ft_skeleton,
          std::forward<X1>(x1), std::forward<X2>(x2));
      }
      else {
        auto ft_skeleton =
          compose<tags::inline_flow>(
            v0 << zip<2>(make_cffts<false>(cffts1<val_t>(-1, m_config.dims[2],
                  info, m_config))) | (input0, input0),
            v1 << zip<3>(make_cffts<true>(cffts2<val_t>(-1, m_config.dims[1],
                  info, m_config))) | (v0, input0, input0),
            v2 << zip<3>(make_cffts<true>(cffts3<val_t>(-1, m_config.dims[0],
                  info, m_config))) | (v1, input0, input1)
          );
        skeletons::execute(
          skeletons::execution_params(default_coarsener()),
          ft_skeleton,
          std::forward<X1>(x1), std::forward<X2>(x2));
      }
    }
    else if (m_config.layout == layout_1d) {
      typedef lightweight_vector<val_t> alltoall_data_t;
      std::size_t d0, d1;
     //if it is a xy_z transformation
      if (m_is_forward) {
        std::size_t l1 = 1;
        d0 = m_config.dims[l1][0] * m_config.dims[l1][1];
        d1 = m_config.dims[l1][2];
      } else {
        std::size_t l1 = 2;
        d0 = m_config.dims[l1][0];
        d1 = m_config.dims[l1][1] * m_config.dims[l1][2];
      }
      //////////////////////////////////////////////////////////////////////
      /// input = [u0_c u1_c]
      /// v0 = map(prepare) [u0_c]
      /// v1 = alltoall<alltoall_data_t, tags::pairwise_exchange>() [v0]
      /// v2 = map(finish) [v1 u1_c]
      /// output = [v2]
      //////////////////////////////////////////////////////////////////////
      auto transpose_skeleton =
        compose<tags::inline_flow>(
          v0 << skeletons::zip<3>(make_cffts<true>(prepare_transpose<val_t>(
                d0, d1, m_config.np2))) | (input0,input1,input2),
          v1 << alltoall<alltoall_data_t, skeletons::tags::pairwise_exchange>()
                | v0,
          v2 << skeletons::zip<3>(transpose_finish(d0, d1)) |
                (v1, input1, input2)
        );

      if (m_is_forward) {
        auto ft_skeleton =
          compose<tags::inline_flow>(
            v0 << zip<2>(make_cffts<false>(cffts1<val_t>(1, m_config.dims[0],
                                         info, m_config))) | (input0, input0),
            v1 << zip<3>(make_cffts<true>(cffts2<val_t>(1, m_config.dims[1],
                                        info, m_config))) | (v0,input0,input0),
            v2 << transpose_skeleton | (v1, input0, input1),
            v3 << zip<3>(make_cffts<true>(cffts1<val_t>(1, m_config.dims[2],
                                        info, m_config))) | (v2, input1, input1)
          );
        skeletons::execute(
          skeletons::execution_params(default_coarsener()),
          ft_skeleton,
          std::forward<X1>(x1), std::forward<X2>(x2));
      } else {
        auto ft_skeleton =
          compose<tags::inline_flow>(
            v0 << zip<2>(make_cffts<false>(cffts1<val_t>(-1, m_config.dims[2],
                                         info, m_config))) | (input0, input0),
            v1 << transpose_skeleton | (v0, input0, input1),
            v2 << zip<3>(make_cffts<true>(cffts1<val_t>(-1, m_config.dims[1],
                                        info, m_config))) | (v1,input1,input1),
            v3 << zip<3>(make_cffts<true>(cffts2<val_t>(-1, m_config.dims[0],
                                        info, m_config))) | (v2, input1, input1)
          );
        skeletons::execute(
          skeletons::execution_params(default_coarsener()),
          ft_skeleton,
          std::forward<X1>(x1), std::forward<X2>(x2));
      }

    } else {
      stapl::abort("2D layout not supported");
    }
  }

  void define_type(typer& t)
  {
    t.member(m_is_forward);
    t.member(m_roots_of_unity);
    t.member(m_config);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Stores the timers for different phases of the FT benchmark.
//////////////////////////////////////////////////////////////////////
struct fft_timers
{
  std::array<double, 6> timers;

  double& checksum_time() { return timers[0]; }
  double& evolve_time()   { return timers[1]; }
  double& fft_time()      { return timers[2]; }
  double& init_time()     { return timers[3]; }
  double& setup_time()    { return timers[4]; }
  double& total_time()    { return timers[5]; }

  void define_type(typer& t)
  {
    t.member(timers);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reduces the timers of the FT benchmark given an operation
/// @c Op.
//////////////////////////////////////////////////////////////////////
template <typename Op>
struct reduce_timers
{
  typedef fft_timers result_type;

  template <typename V1, typename V2>
  result_type operator()(V1&& v1, V2&& v2)
  {
    fft_timers t1 = v1;
    fft_timers t2 = v2;
    for (std::size_t i = 0; i < t1.timers.size(); ++i) {
      t1.timers[i] = Op()(t1.timers[i], t2.timers[i]);
    }
    return t1;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Prints timers in accordance to the output of the NAS FT
/// reference implementation.
//////////////////////////////////////////////////////////////////////
void print_timers(bool verified,
                  double flops,
                  fft_timers total_times,
                  problem_config const& c,
                  problem_traits const& t)
{
  std::cout << "FT Benchmark Completed." << std::endl;
  using namespace std;
  cout << "Class           = " << std::setw(25) << t.clazz << endl;
  cout << "Size            = " << std::setw(25) << std::string(
                                                   std::to_string(t.nx) + "x" +
                                                   std::to_string(t.nx) + "x" +
                                                   std::to_string(t.nx)) <<endl;
  cout << "Iterations      = " << std::setw(25) << t.niters << endl;
  cout << "Time in seconds = " << std::setw(25) << total_times.total_time()/c.np
                                                << endl;
  cout << "Total processes = " << std::setw(25) << c.np << endl;
  cout << "Mop/s total     = " << std::setw(25) << flops << endl;
  cout << "Mop/s/process   = " << std::setw(25) << flops/c.np << endl;
  cout << "Operation type  = " << std::setw(25) << "floating point" << endl;
  cout << "Verification    = " << std::setw(25) << (verified ?
                                                    "SUCCESSFUL" :
                                                    "FAILED") << endl;
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2){
    std::cout << "<exec> <c>" << std::endl;
    exit(1);
  }

  char clazz = argv[1][0];

  typedef double                                    inner_value_t;
  typedef std::complex<inner_value_t>               value_t;
  typedef stapl::static_array<value_t>              complex_array_t;
  typedef stapl::static_array<inner_value_t>        double_array_t;
  typedef stapl::static_array<fft_timers>           timers_array_t;

  problem_traits const traits(clazz);
  problem_config const config(stapl::get_location_id(),
                              stapl::get_num_locations(),
                              traits);

  /// Placeholder for the local checksums
  std::vector<value_t> sums(traits.niters+1);

  /// Use separate timers for each phase of the benchmark
  timers_array_t timers_array(get_num_locations());
  auto timers = make_array_view(timers_array);

  std::size_t me = stapl::get_location_id();
  fft_timers my_timer;


  /// Containers for the initial values, solution, and temporary storage
  std::size_t n = traits.nx*traits.ny*traits.nz;
  complex_array_t u0_container(n);
  complex_array_t u1_container(n);
  complex_array_t u2_container(n);
  auto u0 = make_array_view(u0_container);
  auto u1 = make_array_view(u1_container);
  auto u2 = make_array_view(u2_container);

  /// Container for twiddle factors, precomputed values used in all stages of
  /// the FT benchmark.
  double_array_t twiddle_factors_container(n);
  auto twiddle_factors = make_array_view(twiddle_factors_container);

  using namespace skeletons;
  using skeletons::map;

  /// Skeletons used for the stage of computation except the FFT phase
  auto indexmap_s          = map(compute_indexmap(config));
  auto initial_condition_s = coarse(map(compute_initial_conditions(config)));
  auto evolve_s            = zip<3>(evolve(config.dims[0]));

  /// Run everything once before the timed section
  stapl::counter<stapl::default_timer> local_timer;
  local_timer.start();

  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    indexmap_s,
    twiddle_factors);

  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    initial_condition_s,
    native_view(u1));

  auto&& u = fft_init<value_t>(stapl::get<0>(config.dims[0]));
  fft<value_t>(1, u, config)(u1, u0);
  my_timer.init_time() += local_timer.stop();
  if (me == 0) {
    std::cout << "Initialization Time = " << my_timer.init_time() << std::endl;
  }

  // Start over from the beginning. Note that all operations must
  // be timed, in contrast to other benchmarks.
  stapl::counter<stapl::default_timer> tmr;
  // barrier to get all the timers in sync.
  rmi_fence();
  tmr.start();

  local_timer.reset();
  local_timer.start();

  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    indexmap_s,
    twiddle_factors);

  skeletons::execute(
    skeletons::execution_params(default_coarsener()),
    initial_condition_s,
    native_view(u1));

  u = fft_init<value_t>(stapl::get<0>(config.dims[0]));
  my_timer.setup_time() += local_timer.stop();

  local_timer.reset();
  local_timer.start();
  fft<value_t>(1, u, config)(u1, u0);
  my_timer.fft_time() += local_timer.stop();

  for (int_t i = 1; i <= traits.niters; ++i) {
    local_timer.reset();
    local_timer.start();

    skeletons::execute(
      skeletons::execution_params(default_coarsener()),
      evolve_s,
      u0, u1, twiddle_factors);

    my_timer.evolve_time() += local_timer.stop();

    local_timer.reset();
    local_timer.start();
    fft<value_t>(-1, u, config)(u1, u2);
    my_timer.fft_time() += local_timer.stop();


    local_timer.reset();
    local_timer.start();
    sums[i] = stapl::map_reduce<tags::with_coarsened_wf>(
                checksum<value_t>(config, config.dims[0]),
                stapl::plus<value_t>(), u2);
    my_timer.checksum_time() += local_timer.stop();

    if (me == 0) {
      std::cout << "T = "        << std::setw( 5) << i << ", "
                << "Checksum = " << std::setw(10) << sums[i]
                << std::endl;
    }
  }

  bool verified = verify(traits, sums);
  my_timer.total_time() = tmr.stop();

  timers[me] = my_timer;

  /// Reduce the timers to find out the average, min, and max.
  typedef stapl::plus<double> red_op_t;
  fft_timers average_times = stapl::reduce(timers, reduce_timers<red_op_t>());

  stapl::do_once([&](){
    double mflops = 1.0e-6*config.ntotal_f *
                   (14.8157+7.19641*log(config.ntotal_f)
                +  (5.23518+7.21113*log(config.ntotal_f))*traits.niters)
                   / (average_times.total_time()/config.np);

    std::cout << "Result verification ";
    if (verified) {
      std::cout << "successful" << std::endl;
    }
    else {
      std::cout << "failed";
    }
    std::cout << "class = " << clazz << "\n\n" << std::endl;
    print_timers(verified, mflops, average_times, config, traits);
    std::cout << std::endl;
  });

  return EXIT_SUCCESS;
}


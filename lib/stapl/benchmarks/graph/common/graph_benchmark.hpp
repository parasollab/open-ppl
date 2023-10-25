/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_GAP_GRAPH_BENCHMARK_HPP
#define STAPL_BENCHMARKS_GAP_GRAPH_BENCHMARK_HPP

#include <stapl/utility/do_once.hpp>

#include "../../utilities/observation.hpp"
#include "../../utilities/confint.hpp"

//////////////////////////////////////////////////////////////////////
/// @brief Verifier that unconditionally returns true;
//////////////////////////////////////////////////////////////////////
struct null_verifier
{
  template<typename T>
  bool operator()(T&&) const
  {
    return true;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Graph benchmark object that can be used to benchmark
///        a specific graph algorithm. See @c graph_benchmark_builder for
///        details about the different functions.
//////////////////////////////////////////////////////////////////////
template<typename Setup, typename Runner, typename StatsCalculator,
         typename Verifier>
class graph_benchmark
{
  using setup_data_type = decltype(std::declval<Setup>()());
  using runner_result_type = decltype(
      std::declval<Runner>()(0, std::declval<setup_data_type>()));

  using is_null_verifier =
    typename std::is_same<Verifier, null_verifier>::type;

  int m_num_trials;
  Setup m_setup;
  Runner m_runner;
  StatsCalculator m_stats_calculator;
  Verifier m_verifier;
  confidence_interval_controller m_time_controller;

public:
  graph_benchmark(int trials, Setup setup, Runner runner,
                  StatsCalculator stats_calculator, Verifier verifier)
    : m_num_trials(trials), m_setup(std::move(setup)),
      m_runner(std::move(runner)),
      m_stats_calculator(std::move(stats_calculator)),
      m_verifier(std::move(verifier)),
      m_time_controller(trials, trials)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Benchmark the algorithm and print all information to the given
  ///        output stream.
  //////////////////////////////////////////////////////////////////////
  void operator()(std::ostream& os)
  {
    stapl::do_once([&](){
      os << "Starting benchmark setup" << std::endl;
    });

    auto setup = m_setup();

    int trial = 0;
    do {
      stapl::do_once([&](){
        os << "Starting trial " << trial << std::endl;
      });

      auto res = run_single_trial(trial, setup);

      auto stats = m_stats_calculator(res);

      stapl::do_once([&](){
        os << "Stats for trial " << trial << ": " << stats << std::endl;
      });

      this->verify(stats, trial, os, is_null_verifier{});

      trial++;
    } while (m_time_controller.iterate());

    report(os);
  }

private:
  template<typename SetupData>
  runner_result_type run_single_trial(int trial, SetupData const& setup_data)
  {
    stapl::counter<stapl::default_timer> tmr;

    tmr.start();

    auto res = m_runner(trial, setup_data);

    double t = tmr.stop();
    m_time_controller.push_back(t);

    return res;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Warn that this is benchmark does not have validation.
  //////////////////////////////////////////////////////////////////////
  template<typename T>
  void verify(T&& stats, int trial, std::ostream& os, std::true_type)
  {
    stapl::do_once([&](){
      os << "WARNING: not verifying trial" << std::endl;
    });
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Verifiy benchmark
  //////////////////////////////////////////////////////////////////////
  template<typename Stats>
  void verify(Stats&& stats, int trial, std::ostream& os, std::false_type)
  {
    stapl::do_once([&](){
      os << "Verifying trial " << trial << std::endl;
    });

    if (!m_verifier(std::forward<Stats>(stats)))
      stapl::abort("Verification failed");
  }

  void report(std::ostream& os)
  {
    stapl::do_once([&]() {
      stapl::observation o;

      auto stats = m_time_controller.stats();

      o("time_mean", stats.avg);
      o("time_conf", stats.conf_interval);
      o("time_min", stats.min);
      o("time_max", stats.max);
      o("time_stddev", stats.stddev);

      os << o << '\n';
    });
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Builder to build a graph benchmark object that can be used to
///        benchmark a specific graph algorithm.
///
///        The benchmark object will consist of multiple functions to
///        set up, run, computate statistics of a run and verify
///        correctness. The functions will require the following
///        signature:
///        - setup: () -> T. This function can return any
///          arbitrary type that will be used in subsequent functions.
///
///        - runner: (int, T const&) -> U. This function will receive the trial
///          id and the value from the set up. It will invoke the algorithm
///          to benchmark and return the algorithm's output.
///
///        - stats_calculator: (U) -> V. This function will receive the output
///          of the algorithm and compute some statistics about its execution.
///          The statistics (V) will be printed after each trial, so V should
///          be printable.
///
///        - verifier: (V) -> bool. This function will receive the statistics
///          and return whether or not this run is a valid execution. If the
///          execution is not valid, the program will abort.
//////////////////////////////////////////////////////////////////////
template<typename Setup, typename Runner, typename StatsCalc,
         typename Verifier>
struct graph_benchmark_builder
{
  int m_trials;
  Setup m_setup;
  Runner m_runner;
  StatsCalc m_stats;
  Verifier m_verifier;

public:
  graph_benchmark_builder(void) = default;

  graph_benchmark_builder(int trials, Setup setup, Runner runner,
                  StatsCalc stats, Verifier verifier)
    : m_trials(trials), m_setup(std::move(setup)),
      m_runner(std::move(runner)),
      m_stats(std::move(stats)),
      m_verifier(std::move(verifier))
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the set up function for the benchmark.
  //////////////////////////////////////////////////////////////////////
  template<typename S>
  graph_benchmark_builder<S, Runner, StatsCalc, Verifier>
  with_setup(S&& s)
  {
    return {
      this->m_trials, std::forward<S>(s), std::move(this->m_runner),
      std::move(this->m_stats), std::move(this->m_verifier)
    };
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the runner function for the benchmark.
  //////////////////////////////////////////////////////////////////////
  template<typename R>
  graph_benchmark_builder<Setup, R, StatsCalc, Verifier>
  with_runner(R&& r)
  {
    return {
      this->m_trials, std::move(this->m_setup), std::forward<R>(r),
      std::move(this->m_stats), std::move(this->m_verifier)
    };
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the stats calculator function for the benchmark.
  //////////////////////////////////////////////////////////////////////
  template<typename C>
  graph_benchmark_builder<Setup, Runner, C, Verifier> with_stats(C&& c)
  {
    return {
      this->m_trials, std::move(this->m_setup), std::move(this->m_runner),
      std::forward<C>(c), std::move(this->m_verifier)
    };
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the verifier function for the benchmark.
  //////////////////////////////////////////////////////////////////////
  template<typename V>
  graph_benchmark_builder<Setup, Runner, StatsCalc, V>
  with_verifier(V&& v)
  {
    return {
      this->m_trials, std::move(this->m_setup), std::move(this->m_runner),
      std::move(this->m_stats), std::forward<V>(v)
    };
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set the number of trials for the benchmark.
  //////////////////////////////////////////////////////////////////////
  graph_benchmark_builder<Setup, Runner, StatsCalc, Verifier>
  with_trials(int trials)
  {
    return {
      trials, std::move(this->m_setup), std::move(this->m_runner),
      std::move(this->m_stats), std::move(this->m_verifier)
    };
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Build the benchmark object with the current configuration
  //////////////////////////////////////////////////////////////////////
  graph_benchmark<Setup, Runner, StatsCalc, Verifier> build()
  {
    return {
      this->m_trials, std::move(this->m_setup), std::move(this->m_runner),
      std::move(this->m_stats), std::move(this->m_verifier)
    };
  }
};

using benchmark_builder = graph_benchmark_builder<int, int, int, null_verifier>;

#endif

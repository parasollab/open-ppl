/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/utility/for_each_range.hpp>

#include <benchmarks/utilities/observation.hpp>
#include <benchmarks/utilities/confint.hpp>

#include <vector>
#include <random>
#include <algorithm>


//////////////////////////////////////////////////////////////////////
/// @brief Compute the largest prime factor of a given number
//////////////////////////////////////////////////////////////////////
unsigned largest_prime_factor(unsigned n)
{
  if (n <= 2)
    return n;

  unsigned i = 2;

  for (; i <= n; i++) {
    if (n % i == 0) {
      n /= i;
      i--;
    }
  }

  return i;
}


//////////////////////////////////////////////////////////////////////
/// @brief Given a sequence of numbers, find the length of the longest
///        run of numbers that share the same largest prime factor,
///        using the for_each_range construct
//////////////////////////////////////////////////////////////////////
template<typename V>
std::size_t longest_run_for_each_range(V const& v)
{
  using iterator = typename V::const_iterator;

  long longest_run = 0;

  stapl::utility::for_each_range(v.begin(), v.end(),
    [&longest_run](iterator b, iterator e) {
      longest_run = std::max(longest_run, std::distance(b, e));
    },
    [](unsigned x, unsigned y) {
      return largest_prime_factor(x) == largest_prime_factor(y);
    }
  );

  return longest_run;
}


//////////////////////////////////////////////////////////////////////
/// @brief Given a sequence of numbers, find the length of the longest
///        run of numbers that share the same largest prime factor,
///        using the for_each_range_by construct
//////////////////////////////////////////////////////////////////////
template<typename V>
std::size_t longest_run_for_each_range_by(V const& v)
{
  using iterator = typename V::const_iterator;

  long longest_run = 0;

  stapl::utility::for_each_range_by(v.begin(), v.end(),
    [&longest_run](iterator b, iterator e, unsigned factor) {
      longest_run = std::max(longest_run, std::distance(b, e));
    },
    [](unsigned x) {
      return largest_prime_factor(x);
    }
  );

  return longest_run;
}


//////////////////////////////////////////////////////////////////////
/// @brief Benchmark a specific function and print out the timing information
//////////////////////////////////////////////////////////////////////
template<typename F>
void run_benchmark(std::string version, F&& f)
{
  confidence_interval_controller ci{32, 32};
  std::size_t result;

  do {
    stapl::counter<stapl::default_timer> tmr;
    tmr.start();

    result = f();

    double t = tmr.stop();
    ci.push_back(t);

  } while (ci.iterate());

  auto stats = ci.stats();

  stapl::observation o;

  o("version", version);
  o("run_length", result);
  o("time_mean", stats.avg);
  o("time_conf", stats.conf_interval);
  o("time_min", stats.min);
  o("time_max", stats.max);
  o("time_stddev", stats.stddev);

  std::cout << o << '\n';
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  using vector_type = std::vector<unsigned>;

  std::size_t n = std::atol(argv[1]);

  // Generate random numbers between 1 and n
  std::mt19937 gen{0};
  std::uniform_int_distribution<> dis(1, n);

  vector_type v(n);
  std::generate(v.begin(), v.end(), [&]() {
    return dis(gen);
  });

  // Run the two benchmarks
  run_benchmark("for_each_range", [&v]() {
    return longest_run_for_each_range(v);
  });

  run_benchmark("for_each_range_by", [&v]() {
    return longest_run_for_each_range_by(v);
  });

  return EXIT_SUCCESS;
}

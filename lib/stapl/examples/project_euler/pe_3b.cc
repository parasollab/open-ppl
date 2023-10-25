/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

////////////////////////////////////////////////////////////////////////////////
/// @file
/// Solution of the Euler Problem #3
/// (<a href="https://projecteuler.net/problem=3" target=_blank>link</a>).
////////////////////////////////////////////////////////////////////////////////

#include <stapl/utility/do_once.hpp>
#include <stapl/array.hpp>
#include <stapl/algorithm.hpp>
#include <boost/lexical_cast.hpp>

////////////////////////////////////////////////////////////////////////////////
/// @brief Returns truncated integer square root of given number.
/// @tparam IntType Integral type of the number.
////////////////////////////////////////////////////////////////////////////////
template <typename IntType>
inline IntType sqrt_int(IntType n)
{
  return static_cast<IntType>( std::sqrt(n) );
}

////////////////////////////////////////////////////////////////////////////////
/// @brief Functor that returns the larger of @p n and @p x/n that is both a
///        factor of @p x and a prime number (0 if neither satisfies these
///        conditions).
/// @tparam IntType Integral type of the dividend @p x.
////////////////////////////////////////////////////////////////////////////////
template <typename IntType>
struct largest_prime_factors_filter
{
  static_assert(std::is_integral<IntType>::value, "Integer required.");

  typedef IntType result_type;

  largest_prime_factors_filter(IntType x)
    : m_x(x)
  { }

  IntType operator()(IntType n)
  {
    if ((m_x % n) != 0)   // n is not a factor of x
      return 0;

    if (is_prime(m_x/n))  // x/n (which is larger than n) is prime
      return m_x/n;

    return is_prime(n) ? n : 0;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_x);
  }

private:
  IntType m_x;

  static bool is_prime(IntType n)
  {
    IntType sqrt_n = sqrt_int(n);

    for (IntType i = 2; i <= sqrt_n; ++i)
    {
      if ((n % i) == 0)
        return false;
    }

    return true;
  }
};

stapl::exit_code stapl_main(int argc, char** argv)
{
  using value_t = std::size_t;

  if (argc < 2)
  {
    stapl::do_once( [] {
      std::cout << "Run as: mpirun -n <ncpu> pe_3b <number>" << std::endl;
    });
    return EXIT_FAILURE;
  }

  stapl::counter<stapl::default_timer> exec_timer;
  exec_timer.start();

  // Read the dividend from command line.
  value_t num = boost::lexical_cast<value_t> (argv[1]);

  // Create a generator of consecutive values from 2 to sqrt(num).
  auto c_vw = stapl::counting_view(sqrt_int(num)-1, 2);

  // Apply the filter that effectively replaces each generated number by zero
  // if it is not prime factor of <num> and accumulate a maximum of these
  // numbers.
  value_t max_prime_factor = stapl::map_reduce(
      largest_prime_factors_filter<value_t>(num),
      stapl::max<value_t>(),
      c_vw);

  double t = exec_timer.stop();

  stapl::do_once( [&] {
    std::cout << "Computation finished (time taken: " << t << " s)."
              << std::endl
              << "Largest prime factor of " << num << " is: "
              << max_prime_factor << std::endl;
  });

  return EXIT_SUCCESS;
}

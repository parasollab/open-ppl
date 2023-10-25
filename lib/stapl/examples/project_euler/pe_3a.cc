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
#include <stapl/views/array_view.hpp>
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
      std::cout << "Run as: mpirun -n <ncpu> pe_3a <number>" << std::endl;
    });
    return EXIT_FAILURE;
  }

  stapl::counter<stapl::default_timer> exec_timer;
  exec_timer.start();

  // Read the dividend from command line.
  value_t num = boost::lexical_cast<value_t> (argv[1]);

  // Create array container for storage and a view into it.
  stapl::array<value_t> a(sqrt_int(num)-1);
  auto a_vw = stapl::make_array_view(a);

  // Fill the array with consecutive values from 2 to sqrt(num).
  stapl::iota(a_vw, 2);

  // Replace numbers in the array that are either not factors of num or not
  // prime by 0; also replace any prime factor x by the greater value num/x if
  // the latter is also prime.
  stapl::transform(a_vw, a_vw, largest_prime_factors_filter<value_t>(num));

  // Compute and print the maximum of all numbers in the array.
  value_t max_prime_factor = stapl::max_value(a_vw);

  double t = exec_timer.stop();

  stapl::do_once( [&] {
    std::cout << "Computation finished (time taken: " << t << " s)."
              << std::endl
              << "Largest prime factor of " << num << " is: "
              << max_prime_factor << std::endl;
  });

  return EXIT_SUCCESS;
}

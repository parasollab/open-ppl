/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/array.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/utility/do_once.hpp>
#include <boost/lexical_cast.hpp>

typedef unsigned long long ulong_type;

////////////////////////////////////////////////////////////////////////////////
/// @brief Determines whether a number is fibonacci or not.
///        If the fibonacci number is even, we then keep the number.
///        If the number is not fibonacci number and even, 0 is returned.
////////////////////////////////////////////////////////////////////////////////
struct not_even_fib
{
  typedef ulong_type result_type;
  template <typename T>
  ulong_type operator()(T i)
  {
    // Statements used to determine if a number is Fibonacci or not is
    //   (5*(N^2))-4 OR (5*(N^2))+4
    //   If the result of either statement is a perfect square number,
    //   Then N is a fibonacci number.
    ulong_type fib_check_1 = 5 * (static_cast <ulong_type>(
      pow(static_cast <long double>(i), 2.0))) - 4;
    ulong_type fib_check_2 = 5 * (static_cast <ulong_type>(
      pow(static_cast <long double>(i), 2.0))) + 4;

    // Square root taken to determine if the number is a perfect square.
    ulong_type root_1 = static_cast <ulong_type>(
      sqrt(static_cast<long double>(fib_check_1)));
    ulong_type root_2 = static_cast <ulong_type>(
      sqrt(static_cast<long double>(fib_check_2)));

    // Checks if the number is even and verifies as a perfect square
    //   number.
    if ((root_1 * root_1 == fib_check_1 ||
         root_2 * root_2 == fib_check_2) && (i%2 == 0))
      return i;

   return 0;
  }
};


stapl::exit_code stapl_main(int argc, char** argv)
{
  ulong_type num = boost::lexical_cast<ulong_type> (argv[1]);

  // Creates a non-storage view in counting order from 1 to n.
  auto view_2 = stapl::counting_view<ulong_type>(num, 1);

  // Maps not_even_fib functor to set numbers in the view that are not
  //   divisible by 3 or 5 to 0. Then stapl::plus is called to add the numbers
  //   in the view.
  ulong_type ans = stapl::map_reduce(
   not_even_fib(), stapl::plus<ulong_type>(), view_2
  );

  // Prints the total sum.
  stapl::do_once([&]{
    std::cout << "The total is: " << ans << std::endl;
  });

  return EXIT_SUCCESS;
}

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
/// @brief Returns true if a given number is NOT divisible by
///        3 or 5 and returns false otherwise.
////////////////////////////////////////////////////////////////////////////////
struct three_five_divisor
{
  template<typename T>
  bool operator()(T i)
  {
    return !((i % 3) == 0 || (i % 5) == 0);
  }
};


stapl::exit_code stapl_main(int argc, char** argv)
{
  ulong_type num = boost::lexical_cast<ulong_type> (argv[1]);

  // Creates array container of unsigned integers that will be used for storage.
  stapl::array<ulong_type> b(num);

  // Creates view over container.
  stapl::array_view<stapl::array<ulong_type>> vw(b);

  // Fills the container with values from 1 to n.
  stapl::iota(vw, 1);

  // For numbers in the container that return true to the three_five_divisor
  //   functor, they are set to 0.
  stapl::replace_if(vw, three_five_divisor(), 0);

  // Adds the total of all elements in container.
  ulong_type total = stapl::accumulate(vw, (ulong_type)0);

  // Prints the total sum.
  stapl::do_once ([&] {
    std::cout << "The total is: " << total << std::endl;
  });

  return EXIT_SUCCESS;
}

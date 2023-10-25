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

///////////////////////////////////////////////////////////////////////////////
/// @brief Determines whether a number is divisible by 3 or 5. If this
///        condition is true, the number that was tested will be
///        returned. Otherwise, 0 will be returned.
///////////////////////////////////////////////////////////////////////////////
struct three_five_divisor
{
  typedef ulong_type result_type;
  template<typename T>
  ulong_type operator()(T const& i)
  {
    if ((i % 3) == 0 || (i % 5) == 0)
      return i;

    return 0;
  }
};


stapl::exit_code stapl_main(int argc, char** argv)
{
  ulong_type num = boost::lexical_cast<ulong_type> (argv[1]);

  // Creates a non-storage view in counting order from 1 to num.
  auto vw = stapl::counting_view<ulong_type>(num, 1);

  // Maps three_five_divisor functor to set numbers in the view that are not
  //   divisible by 3 or 5 to 0. Then stapl::plus is called to add the numbers
  //   in the view.
  ulong_type total = stapl::map_reduce(
    three_five_divisor(), stapl::plus<ulong_type>(), vw
  );

  // Prints the total sum.
  stapl::do_once ([&] {
    std::cout << "The total is: " << total << std::endl;
  });

  return EXIT_SUCCESS;
}

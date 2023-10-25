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

using namespace stapl;

stapl::exit_code stapl_main(int argc, char* argv[])
{

  typedef size_t value_t;
  typedef array_view<array<value_t>> view_type;


  size_t n = atol(argv[1]);
  stapl::array<size_t> c(n);
  stapl::array_view<stapl::array<size_t>> v(c);

  //Fill the array with sequential values, from 1 to n.
  stapl::iota(v,1);

  //Compute the inner product of the array with itself.
  size_t a = stapl::inner_product(v, v, 0ul);

  //Compute the sum of the values in the array.
  size_t b = stapl::accumulate(v, 0ul);

  //Square the sum of the values int the array.
  size_t b2 = b*b;

  //Compute the difference between the inner product and the square of the sum.
  size_t ss = b2 - a;

  stapl::do_once([&]{
    std::cout << ss << " " << std::endl;
  });

  return EXIT_SUCCESS;

}

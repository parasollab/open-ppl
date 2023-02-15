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

struct prime
{
  typedef bool result_type;

  result_type operator()(int i)
  {
    int c = 0;
    for (int a = 1; a <= i; ++a)
    {
      if(i%a==0) c++;
    }

    if (c==2)
    {
      return false;
    }else
    {
      return true;
    }
  }
};

stapl::exit_code stapl_main(int argc, char* argv[])
{

  typedef size_t value_t;
  typedef stapl::array_view<stapl::array<value_t>> view_type;
  size_t n = atol(argv[1]);
  stapl::array<size_t> c(n);
  stapl::array_view<stapl::array<size_t>> v(c);

  //Fill the array with sequential values, from 1 to n.
  stapl::iota(v,1);

  //Replace all array values that are not prime numbers with the value 0.
  stapl::replace_if(v, prime(), 0);

  //Compute the sum of all the values in the array.
  size_t s = stapl::accumulate(v, 0);

  stapl::do_once([&]{
    std::cout << s << std::endl;
  });

  return EXIT_SUCCESS;

}

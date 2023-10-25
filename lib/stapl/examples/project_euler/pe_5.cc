/*
// Copyright (c) 2015, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <algorithm>
#include <utility>
#include <time.h>
#include <stapl/utility/do_once.hpp>
#include <stapl/array.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/algorithms/functional.hpp>


using namespace stapl;


struct divid
{
  template <typename T>
  long operator() (T x)
  {
    int counter = 0;

    for (int i = 1; i < 21; ++i)
    {
      if (x%i == 0)
      {
        counter++;
      }
    }
    if (counter == 20)
    {
      return x;
    } else {
      return std::numeric_limits<long>::max();
    }
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  long i = 1;
  long result = 1;
  long n = 5000000;

  do {
    auto vw = counting_view(n,i);

    result = map_reduce(divid(), stapl::min<long> (), vw);

    if (result != std::numeric_limits<long>::max())
    {
      stapl::do_once( [&] {
        std::cout << " result is "<< result << std::endl;
      });
    }

    // double the size of the range for the next  map_reduce call.
    i=n+1;
    n+=n;
  } while (result == std::numeric_limits<long>::max());

  return EXIT_SUCCESS;
}

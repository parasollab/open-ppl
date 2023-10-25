/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
#include <iostream>
#include <stapl/views/counting_view.hpp>
#include <stapl/skeletons/map_reduce.hpp>
#include <stapl/algorithms/functional.hpp>
#include "../../test/confint.hpp"

typedef unsigned long long ulong_type;

typedef stapl::counter<stapl::default_timer> counter_t;

// Function to determine if a given i is a palindrome in the specified base.
bool palindrome(ulong_type i, int base)
{
  ulong_type reversed = 0;
  ulong_type num = i;
  while ( num > 0 )
  {
    reversed = base*reversed + num%base;
    num /= base;
  }
  if (i != reversed)
    return false;
  else
    return true;
}

struct palindrome_number
{
  typedef ulong_type result_type;
  template<typename T>
  ulong_type operator()(T const& i)
  {
    if (!palindrome(i,10))
      return 0;
    else
    {
      if (!palindrome(i,2))
        return 0;
      else
        return i;
    }
  }
}; // struct palindrome_number.

template<typename T, typename Policy>
void interval(T n, T& acc, confidence_interval_controller& controller)
{
  // Create timer.
  counter_t count;

  //Iterate until the controller has collected the maximum number of samples
  //or the confidence interval is smaller than the specified percentage.
  while (controller.iterate())
  {
    // Reset count.
    count.reset();

    // Start count.
    count.start();

    // Create halved counting view.
    auto vw = stapl::counting_view <T, Policy>(n, 1);

    // Map operation returns the value of all numbers that are
    // palindromes base 10 and 2, and 0 otherwise.
    // The reduce operation computes the sum of the results.
    acc  = stapl::map_reduce(
      palindrome_number(), stapl::plus<T>(), vw);

    // Stop count.
    count.stop();

    // Add the execution time to confidence interval controller.
    controller.push_back(count.value());
  }
}

stapl::exit_code stapl_main(int argc, char **argv)
{
  // Size of the counting_view.
  ulong_type n = boost::lexical_cast<ulong_type>(argv[1]);

  // Option which indicates the counting_view to use:
  //   1: Halved counting_view.
  //   2: Cyclic counting_view.
  //   3: Interleaved counting_view.
  int option = boost::lexical_cast<int>(argv[2]);

  // Accumulator.
  ulong_type acc;

  // Number of samples to calculate the confidence interval.
  const int samples = 32;

  // Create controller of confidence interval.
  confidence_interval_controller controller(samples, samples, 0.05);

  switch (option)
  {
    case 1:
    {
      interval<ulong_type, stapl::view_impl::halved_container>
        (n, acc, controller);
      break;
    }
    case 2:
    {
      interval<ulong_type, stapl::view_impl::cyclic_container>
        (n, acc, controller);
      break;
    }
    case 3:
    {
      interval<ulong_type, stapl::view_impl::interleaved_container>
        (n, acc, controller);
      break;
    }
    default:
    {
      interval<ulong_type, stapl::view_impl::default_container>
        (n, acc, controller);
      break;
    }
  }

  // Executes the instructions only one time.
  stapl::do_once ([&] {

    // Print the report with the confidence interval of
    // the execution time.
    controller.report("Project Euler 36");

    // Print the Results.
    std::cout << "Result: N: " << n << " Sum: " << acc << std::endl;
    });

  return EXIT_SUCCESS;
}

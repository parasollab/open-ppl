/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <vector>
#include <algorithm>
#include <utility>
#include <stapl/array.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/runtime/counter/default_counters.hpp>

using namespace stapl;

// work function is used to check whether a, b, c make a Pythagorean triple
struct inner
{
  typedef int value_type;
  typedef int result_type;

  value_type m_a;

  inner(value_type a)
    : m_a(a)
  {}

  template <typename Value>
  result_type operator()(Value B)
  {
    // The problem states a, b, and c must add up to 1000
    value_type C = 1000 - m_a - B;

    if (((m_a*m_a) + (B*B)) == (C*C))
      return m_a*B*C;
    else
      return 0;
  }

  void define_type(typer&t)
  {
    t.member(m_a);
  }
};

// work function stores the value of a and calls a map reduce in order to nest
// the map_reduce functions so every value of a can be tested with every value
// of b
struct outer
{
  typedef int result_type;

  template <typename Value>
  result_type operator()(Value A)
  {
    inner i_f(A);

    // represented in pseudocode as a for loop from 1 to 500
    return map_reduce(i_f,stapl::max<int>(),stapl::counting_view<int>(500,1));
  }
};


stapl::exit_code stapl_main(int argc, char **argv)
{
  stapl::counter<stapl::default_timer> timer;
  timer.start();

  // Represented in pseudocode as a for loop from 1 to 1000
  auto a = map_reduce(outer(), stapl::max<int>(),
             stapl::counting_view<int>(1000,1));

  auto time = timer.stop();
  stapl::do_once([=]{std::cout << time << std::endl;});
  stapl::do_once([=]{std::cout << a << std::endl;});
  return EXIT_SUCCESS;
}

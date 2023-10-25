/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/list/list.hpp>
#include <stapl/views/list_view.hpp>
#include "../../test_report.hpp"

template<typename T>
class sequence
{
private:
  T counter, oldCtr, step;

public:
  using state_type = std::true_type;

  sequence(T start = 0, T st = 1)
    : counter(start), oldCtr(start), step(st) {}

  sequence(sequence const& seq, std::size_t offset)
    : counter(seq.counter+offset * seq.step), oldCtr(counter), step(seq.step)
  { }

  void define_type(stapl::typer& t)
  {
    t.member(counter);
    t.member(oldCtr);
    t.member(step);
  }

  T operator()(void)
  {
    oldCtr = counter;
    counter += step;
    return oldCtr;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef stapl::list<int>                 container_type;
  typedef stapl::list_view<container_type> view_type;

  if (argc != 2)
  {
    printf("usage: list_algos n\n");
    return EXIT_FAILURE;
  }

  //Create container and view
  std::size_t n = atol(argv[1]);
  container_type c1(n);
  view_type v1(c1);

  //Check the container size.
  bool size_passed = c1.size() == n;
  STAPL_TEST_REPORT(size_passed,"Testing list constuctor of a given size");

  //Fill containers with sequence [1, n]
  stapl::generate(v1,sequence<int>(1,1));

  // The sum of elements should be n(n+1)/2.
  int sum = stapl::accumulate(v1, 0);

  bool sum_passed = sum == (int)(n*(n+1)/2);
  STAPL_TEST_REPORT(sum_passed,"Testing algorithms on list");

  return EXIT_SUCCESS;
}

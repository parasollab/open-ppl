/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/utility/tuple.hpp>
#include <stapl/runtime.hpp>
#include <stapl/paragraph/paragraph.hpp>
#include <stapl/algorithms/dynamic_programming.hpp>
#include <stapl/containers/array/static_array.hpp>
#include <stapl/views/cross_view.hpp>
#include <vector>
#include "../../test_report.hpp"

template <typename T>
struct dp_plus_wf
{
  typedef T result_type;

  // Operator for the first subproblem
  //
  template <typename ElementTuple>
  result_type operator()(ElementTuple e) const
  {
    typename ElementTuple::value_type const& tuple_ref = e;

    return stapl::get<0>(tuple_ref)
           * stapl::get<1>(tuple_ref);
  }


  // Operator for subproblems in first row and column
  //
  template <typename ElementTuple, typename Res1>
  result_type operator()(ElementTuple e, Res1 r1) const
  {
    typename ElementTuple::value_type const& tuple_ref = e;

    return r1
      + stapl::get<0>(tuple_ref)
      * stapl::get<1>(tuple_ref);
  }


  // Operator for the remaining subproblems
  //
  template <typename ElementTuple,
            typename Res1, typename Res2, typename Res3>
  result_type operator()(ElementTuple e, Res1 r1, Res2 r2, Res3 r3) const
  {
    typename ElementTuple::value_type const& tuple_ref = e;

    return r1 + r2 + r3
           + stapl::get<0>(tuple_ref)
           * stapl::get<1>(tuple_ref);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef stapl::static_array<long long int>  cont_1D_type;
  typedef stapl::array_view<cont_1D_type>     view_1D_type;

  stapl::counter<stapl::default_timer> timer;

  cont_1D_type a(16, 1);
  cont_1D_type b(16, 1);

  view_1D_type va(a);
  view_1D_type vb(b);

  auto cv = stapl::make_cross_view(va, vb);

  typedef stapl::sequence_comp_factory<dp_plus_wf<long long int> > factory_type;

  timer.reset();
  timer.start();

  std::vector<std::vector<long long int> > result =
    stapl::paragraph<
      stapl::default_scheduler,
      stapl::sequence_comp_factory<dp_plus_wf<long long int>>,
      decltype(cv)
    >(factory_type(dp_plus_wf<long long int>(), 4), cv)
    ();
  double exec_time = timer.stop();

  stapl::do_once([&exec_time, &result]() {
    printf("Test: dynamic_programming\nStatus: ");
    if (result[2][0] == 126027618304LL)
      printf("PASS");
    else
      printf("FAIL");
    printf("\nVersion: stapl\ntime: %f\n", exec_time);
  });
  return EXIT_SUCCESS;
}

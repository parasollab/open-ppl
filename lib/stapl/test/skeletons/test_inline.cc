/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <stapl/array.hpp>
#include <stapl/skeletons/operators/compose.hpp>
#include <stapl/skeletons/functional/zip.hpp>
#include <stapl/skeletons/functional/sink.hpp>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/views/metadata/coarseners/null.hpp>
#include <stapl/utility/do_once.hpp>
#include "../test_report.hpp"

using namespace stapl;

template<class ValueType>
struct adder {
  using return_type = ValueType;

  template<class T, class U>
  return_type operator()(T t, U u) const {
    return t + u;
  }
  template<class T, class U, class... Us>
  return_type operator()(T t, U u, Us&&... us) const {
    return (*this)(t+u, std::forward<Us>(us)...);
  }
};

template<typename... Ts>
void no_op(Ts...){}

exit_code stapl_main(int argc, char** argv)
{

  using namespace stapl::skeletons;

  if (argc < 2) {
    std::cout<< "usage: test_inline <n>" <<std::endl;
    return EXIT_FAILURE;
  }
  const std::size_t n = atol(argv[1]);

  using value_type = size_t;
  using op = adder<value_type>;

  array<value_type> left_array(n, n);
  array<value_type> right_array(n, n/2);
  array<value_type> output_array(n);

  auto left_view = make_array_view(left_array);
  auto right_view = make_array_view(right_array);
  auto output_view = make_array_view(output_array);

  const value_type expected = 2*n + n/2;

  // Declare special variables we can use to use as placeholders when
  // specifying the flow
  DECLARE_INLINE_INPUT_PLACEHOLDERS(2, in)
  DECLARE_INLINE_PLACEHOLDERS(3, x)

  auto s = compose<tags::inline_flow>(
      x0 << zip(op{}) | (in0, in1),
      x1 << zip(op{}) | (x0, x0), //output is ignored
      x2 << zip(op{}) | (in0, x0)
  );


  // We can also manually make the placeholder variables, like so
  using namespace flows::inline_flows::placeholders;
  input<0> left;
  input<1> right;

  // Transpose the skeletons and check the self-sorting version works
  using Sorted = decltype(compose<tags::sorted_inline_flow>(
        x1 << zip(op{}) | (x0, x0),
        x0 << zip(op{}) | (left, right),
        x2 << zip(op{}) | (left, x0)
        ));

  static_assert(std::is_same<decltype(s), Sorted>(),
      "Using tags::sorted_inline_flow should use a (stable) topological sort");


  skeletons::execute(
    skeletons::default_execution_params(),
    sink<value_type>(std::move(s)),
    left_view, right_view, output_view);

  bool passed = true;

  stapl::do_once([&]{
    for (auto&& e : output_view) {
      if (e != expected) {
        passed = false;
        break;
      }
    }
  });

  STAPL_TEST_REPORT(passed, "Testing basic compose with inline flows");

  return EXIT_SUCCESS;
}

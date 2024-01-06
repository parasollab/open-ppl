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
#include <stapl/skeletons/functional/allreduce.hpp>
#include <stapl/skeletons/functional/sink.hpp>
#include <stapl/skeletons/functional/stencil.hpp>
#include <stapl/views/metadata/coarseners/null.hpp>
#include <stapl/utility/do_once.hpp>
#include "../test_report.hpp"
#include <stapl/skeletons/transformations/compose/zip_fusion.hpp>

#include <stapl/runtime/counter/default_counters.hpp>

#define RUN_SKELETON

using namespace stapl;

template<class ValueType>
struct adder
{
  using return_type = ValueType;

  template<class T, class U>
  return_type operator()(T t, U u) const
  {
    return t + u;
  }
  template<class T, class U, class... Us>
  return_type operator()(T t, U u, Us&&... us) const
  {
    return (*this)(t+u, std::forward<Us>(us)...);
  }
};

template<typename... Ts>
void no_op(Ts...){}


template<typename... Args>
stapl::default_timer::value_type run_timed(Args&&... args)
{
  stapl::counter<stapl::default_timer> timer;

  skeletons::execute(args...);
  timer.start();
  for (unsigned i = 0; i < 20; ++i)
    skeletons::execute(args...);
  return timer.stop();
}

// Simple filter to check that zip fusion filters results correctly
// Note: currently, this should only be used for skeletons that don't from views
// until filtering from views is implemented.
template<class T>
struct negate_filter
{
  T operator()(T const& t) const
  {
    return -t;
  }

  template<class... Ts>
  int configure_filter(Ts...)
  {
    return 0;
  }

  constexpr bool operator==(negate_filter const&) const
  {
    return true;
  }
};

exit_code stapl_main(int argc, char** argv)
{

  using namespace stapl::skeletons;

  if (argc < 2)
  {
    std::cout<< "usage: test_fuse <n>" <<std::endl;
    return EXIT_FAILURE;
  }

  using value_type = size_t;
  using op = adder<value_type>;

  using negative_result_traits =
    skeletons_impl::skeleton_traits<tuple<>, negate_filter<value_type>>;

  DECLARE_INLINE_INPUT_PLACEHOLDERS(4, in)
  DECLARE_INLINE_PLACEHOLDERS(5, x)
  auto s = compose<tags::inline_flow>(
      x0 << zip(op{}) | (in2, in1)
      , x1 << zip(op{}) | (x0, in0)
      , x2 << zip(op{}, negative_result_traits()) | (x1, x0)
      , x3 << zip(op{}) | (x2, in2)
      , x4 << allreduce(op{}) | x3
  );


  using passes = tags::zip_fusion;

  auto sinkS = (compose<tags::inline_flow>(
      x0 << s | (in0, in1, in2),
      x1 << copy<value_type>() | (x0, in3)
  ));

  auto fused = skeletons::transform<passes>(s);

  auto sinkF = (compose<tags::inline_flow>(
      x0 << fused | (in0, in1, in2),
      x1 << copy<value_type>() | (x0, in3)
  ));


  const std::size_t n = atol(argv[1]);

  array<value_type> left_array(n, n);
  array<value_type> right_array(n, n/2);
  array<value_type> output_array(n);
  array<value_type> fused_output_array(n);

  auto left_view = make_array_view(left_array);
  auto right_view = make_array_view(right_array);
  auto count_view = counting_view<value_type>(n, 0);
  auto output_view = make_array_view(output_array);
  auto fused_output_view = make_array_view(fused_output_array);

  auto exec = default_execution_params();

  // Execute the original skeleton
  skeletons::execute(exec, sinkS, left_view, right_view, count_view,
      output_view);

  // And now the fused version
  skeletons::execute(exec, sinkF, left_view, right_view, count_view,
      fused_output_view);

  bool passed = stapl::equal(output_view, fused_output_view);

  STAPL_TEST_REPORT(passed, "Testing zip fusion doesn't change behavior");

  return EXIT_SUCCESS;
}

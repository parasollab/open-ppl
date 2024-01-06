/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/algorithms/paradigms/concurrency_model.hpp>

struct vertex_operator_strong
{
  using concurrency_model = stapl::sgl::strong_concurrency;
};

struct vertex_operator_weak
{
  using concurrency_model = stapl::sgl::weak_concurrency;
};

struct vertex_operator_none
{ };

stapl::exit_code stapl_main(int argc, char* argv[])
{
  static_assert(
    stapl::sgl::has_strong_concurrency_model<vertex_operator_strong>::type::
      value,
    "Strong operator has strong model");

  static_assert(
    !stapl::sgl::has_strong_concurrency_model<vertex_operator_weak>::type::
      value,
    "Weak operator does not have strong model");

  static_assert(
    stapl::sgl::has_strong_concurrency_model<vertex_operator_none>::type::
      value,
    "Default has strong model");

  return EXIT_SUCCESS;
}

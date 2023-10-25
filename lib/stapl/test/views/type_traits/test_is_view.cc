/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/array.hpp>
#include <stapl/views/type_traits/is_view.hpp>

stapl::exit_code stapl_main(int argc, char* argv[])
{
  using array_type = stapl::array<int>;
  using array_view_type = stapl::array_view<array_type>;

  static_assert(!stapl::is_view<array_type>::value, "Array is not a view");
  static_assert(stapl::is_view<array_view_type>::value, "Array view is a view");

  return EXIT_SUCCESS;
}

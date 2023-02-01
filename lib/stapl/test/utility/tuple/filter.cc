/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/utility/tuple/filter.hpp>

using namespace stapl;

exit_code stapl_main(int argc, char** argv)
{
  using lower = std::tuple<
    std::integral_constant<std::size_t, 0ul>,
    std::integral_constant<std::size_t, 2ul>
  >;

  using upper = std::tuple<
    std::integral_constant<std::size_t, 4ul>,
    std::integral_constant<std::size_t, 5ul>,
    std::integral_constant<std::size_t, 6ul>
  >;

  using expected = std::tuple<
    std::integral_constant<std::size_t, 4ul>,
    std::integral_constant<std::size_t, 6ul>
  >;

  using result =
    typename tuple_ops::result_of::heterogeneous_filter<lower, upper>::type;

  static_assert(std::is_same<result, expected>::value,
    "Testing <0,2> <4,5,6>"
  );
  return EXIT_SUCCESS;
}

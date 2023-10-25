/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/views/type_traits/remap_slices.hpp>

using namespace stapl;

void test_3d_2d()
{
  using outer = index_sequence<0,1,2>;
  using current = index_sequence<0,1>;

  using result = remap_slices<outer, current, 5>::type;
  using expected = std::tuple<std::integral_constant<std::size_t, 0ul>>;

  static_assert(std::is_same<result, expected>::value,
    "Testing <0,1,2> and <0,1>"
  );
}

void test_4d_1d()
{
  using outer = index_sequence<0,1,2,3>;
  using current = index_sequence<3>;

  using result = remap_slices<outer, current, 5>::type;
  using expected = std::tuple<
    std::integral_constant<std::size_t, 0ul>,
    std::integral_constant<std::size_t, 1ul>,
    std::integral_constant<std::size_t, 2ul>
  >;

  static_assert(std::is_same<result, expected>::value,
    "Testing <0,1,2,3> and <3>"
  );
}

void test_3d_2d_offset()
{
  using outer = index_sequence<1,2,3>;
  using current = index_sequence<0,1>;

  using result = remap_slices<outer, current, 5>::type;
  using expected = std::tuple<std::integral_constant<std::size_t, 1ul>>;

  static_assert(std::is_same<result, expected>::value,
    "Testing <1,2,3> and <0,1>"
  );
}


void test_4d_2d()
{
  using outer = index_sequence<0,2,4>;
  using current = index_sequence<0,2>;

  using result = remap_slices<outer, current, 5>::type;
  using expected = std::tuple<
    std::integral_constant<std::size_t, 1ul>
  >;

  static_assert(std::is_same<result, expected>::value,
    "Testing <0,2,4> and <0,2>"
  );
}

void test_dzg()
{
  using outer = index_sequence<0,1,2,3>;
  using current = index_sequence<3>;

  using result = remap_slices<outer, current, 5>::type;
  using expected = std::tuple<
    std::integral_constant<std::size_t, 0ul>,
    std::integral_constant<std::size_t, 1ul>,
    std::integral_constant<std::size_t, 2ul>
  >;

  static_assert(std::is_same<result, expected>::value,
    "Testing DZG"
  );
}

exit_code stapl_main(int argc, char** argv)
{
  return EXIT_SUCCESS;
}

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/utility/tuple/rearrange.hpp>
#include <stapl/utility/tuple/from_index.hpp>

using namespace stapl;

stapl::exit_code stapl_main(int, char*[])
{
  using two_tuple_t = stapl::tuple<std::size_t, std::size_t>;

  using tup = tuple_ops::from_index_sequence<index_sequence<0,1,2>>::type;
  using order = tuple_ops::from_index_sequence<index_sequence<1,2,0>>::type;
  using expected = tuple_ops::from_index_sequence<index_sequence<2,0,1>>::type;

  using result = tuple_ops::result_of::rearrange<tup, order>::type;

  static_assert(std::is_same<expected,result>::value, "<0,1,2> and <1,2,0>");

  return EXIT_SUCCESS;
}

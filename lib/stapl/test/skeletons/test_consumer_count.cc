/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <stapl/runtime.hpp>
#include <stapl/utility/tuple/tuple.hpp>
#include <stapl/skeletons/operators/consumer_count.hpp>
#include "../expect.hpp"

template <typename Customizer>
struct flow
{
  Customizer m_customizer;

  template <typename Coord>
  std::size_t consumer_count(Coord&& coord) const
  {
    return m_customizer(coord);
  }
};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  using stapl::tuple;
  using stapl::make_tuple;
  using stapl::get;
  using stapl::skeletons::consumer_count;
  using stapl::tests::expect_eq;

  // Creating a synthetic coordinate
  using coord_t = tuple<int, int>;
  coord_t coord{1, 2};

  // Creating customizers to test various cases of flows
  auto customizer1 = [](coord_t const& coord) {
    return get<0>(coord) + get<1>(coord);
  };

  auto customizer2 = [](coord_t const& coord) {
    return get<0>(coord) * get<1>(coord);
  };

  auto customizer3 = [](coord_t const& coord) {
    return get<0>(coord);
  };

  // Using aggregate initializers to create two synthetic flows
  auto flow1 = flow<decltype(customizer1)>{customizer1};
  auto flow2 = flow<decltype(customizer2)>{customizer2};
  auto flow3 = flow<decltype(customizer3)>{customizer3};

  auto test_flow1 = flow1;
  auto test_flow2 = flow2;
  auto test_flow3 = flow3;
  auto test_flow4 = make_tuple(flow1);
  auto test_flow5 = make_tuple(flow1, flow2);
  auto test_flow6 = make_tuple(flow1, flow2, flow3);
  auto test_flow7 = make_tuple(make_tuple(flow1, flow2), flow3);
  auto test_flow8 = make_tuple(flow1, make_tuple(flow2, flow3));
  auto test_flow9 = make_tuple(flow1, make_tuple(make_tuple(flow2), flow3));

  expect_eq(consumer_count(test_flow1, coord), 3ul) << "test_flow1";
  expect_eq(consumer_count(test_flow2, coord), 2ul) << "test_flow2";
  expect_eq(consumer_count(test_flow3, coord), 1ul) << "test_flow3";
  expect_eq(consumer_count(test_flow4, coord), 3ul) << "test_flow4";
  expect_eq(consumer_count(test_flow5, coord), 5ul) << "test_flow5";
  expect_eq(consumer_count(test_flow6, coord), 6ul) << "test_flow6";
  expect_eq(consumer_count(test_flow7, coord), 6ul) << "test_flow7";
  expect_eq(consumer_count(test_flow8, coord), 6ul) << "test_flow8";
  expect_eq(consumer_count(test_flow9, coord), 6ul) << "test_flow9";

  return EXIT_SUCCESS;
}

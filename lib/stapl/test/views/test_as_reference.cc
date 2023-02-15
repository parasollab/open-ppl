/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/proxy/as_reference.hpp>

template<typename Container>
void test_as_reference(Container& c)
{
  auto& bc = *c.distribution().container_manager().begin();
  const auto first = bc.domain().first();

  auto x = bc[first];
  auto& x_ref = stapl::as_reference(x);

  using value_type = typename std::decay<decltype(bc)>::type::value_type;

  static_assert(std::is_same<decltype(x_ref), value_type&>::type::value,
    "Retrieved raw reference from proxy");
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  constexpr std::size_t n = 10;

  stapl::array<int> a{n};
  stapl::multidigraph<int> g{n};

  test_as_reference(a);
  test_as_reference(g);

  return EXIT_SUCCESS;
}

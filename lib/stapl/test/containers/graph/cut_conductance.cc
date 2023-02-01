/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "test_util.h"
#include "../../test_report.hpp"

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/numeric.hpp>

#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/cut_conductance.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>

struct clear_if
{
  typedef std::size_t result_type;

  template<typename T, typename U>
  result_type operator()(T x, U y)
  {
    if (x > y)
      return 0;
    else
      return x;
  }
};

// create a sequence of blocks of size (n * ratio) starting from 1:
//  [ 1, 1, 1, 2, 2, 2, 3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 ]
template<typename View>
void fill_membership(View& vw, double member_ratio, size_t num_sets)
{
  const std::size_t size = vw.size();
  const std::size_t block = size*member_ratio;

  stapl::iota(vw, block);

  stapl::transform(vw, vw, boost::bind(stapl::divides<size_t>(), _1, block));
  stapl::transform(vw, vw, boost::bind(clear_if(), _1, num_sets));
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 5) {
    std::cout << "usage: " << argv[0] << " x y num_sets ratio" << std::endl;
    exit(1);
  }

  // inputs
  const size_t x = atoi(argv[1]);
  const size_t y = atoi(argv[2]);
  const size_t num_sets = atoi(argv[3]);
  const double member_ratio = atof(argv[4]);

  const size_t n = x*y;

  // set up graph and view
  typedef stapl::multidigraph<> graph_type;
  typedef stapl::graph_view<graph_type> view_type;

  // generate a mesh
  view_type v = stapl::generators::make_mesh<view_type>(x, y);

  // set up map for membership
  typedef stapl::array<std::size_t> member_storage_type;
  typedef stapl::array_view<member_storage_type> member_view_type;
  typedef stapl::graph_external_property_map<view_type, std::size_t,
                                             member_view_type> member_map_type;

  member_storage_type m(n);
  member_view_type members(m);

  // generate membership values
  fill_membership(members, member_ratio, num_sets);

  // create map
  member_map_type member_map(v, members);

  // call conductance for set 0
  double c = stapl::cut_conductance(v, member_map, 0);

  bool passed = c > 0;

  STAPL_TEST_REPORT(passed, "Testing cut conductance")

  return EXIT_SUCCESS;
}

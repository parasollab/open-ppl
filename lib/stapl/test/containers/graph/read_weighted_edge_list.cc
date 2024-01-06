/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "test_util.h"
#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/algorithms/algorithm.hpp>

using namespace stapl;

struct sum_edge_weight
{
  using result_type = std::size_t;

  template<typename Vertex>
  result_type operator()(Vertex v) const
  {
    using edge_type = decltype(*v.begin());

    auto reducer = [](int running, edge_type const& e) {
      return running + e.property();
    };

    return std::accumulate(std::begin(v), std::end(v), 0ul, reducer);
  }
};

exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    std::cerr << "usage: exe filename" << std::endl;
    exit(1);
  }

  std::string filename = argv[1];

  using graph_type = graph<DIRECTED, MULTIEDGES, int, int>;

  auto vw = read_weighted_edge_list<graph_type>(filename);

  std::size_t sum_edges =
    map_reduce(sum_edge_weight{}, stapl::plus<std::size_t>{}, vw);

  bool passed = sum_edges == vw.num_edges() * 2;
  one_print(passed);

  return EXIT_SUCCESS;
}


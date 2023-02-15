/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/breadth_first_search.hpp>
#include <stapl/containers/graph/algorithms/betweenness_centrality.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/list.hpp>

#include "../../test_report.hpp"

using namespace stapl;

struct less_centrality
{
  using result_type = bool;

  template<typename U, typename V>
  result_type operator()(U&& u, V&& v) const
  {
    return u.property().BC() < v.property().BC();
  }
};

stapl::exit_code stapl_main(int argc,char** argv)
{
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " n\n";
    return EXIT_FAILURE;
  }

  const std::size_t n = atol(argv[1]);

  if (n % 2 == 0)
    stapl::abort("Input size must be odd.");

  using graph_type = graph<UNDIRECTED, MULTIEDGES, properties::bc_property>;
  using view_type = graph_view<graph_type>;

  auto g = stapl::generators::make_list<view_type>(n);

  betweenness_centrality(g, 4);

  // The vertex with the highest centrality should be the one directly
  // in the middle of the chain
  auto most_important = stapl::max_element(g, less_centrality{});

  STAPL_TEST_REPORT(most_important.descriptor() == (n-1)/2,
    "Highest centrality");

  // The vertex with the lowest centrality should be either one of the
  // vertices on the end (0 or n-1)
  auto least_important = stapl::min_element(g, less_centrality{});

  std::vector<std::size_t> ends = {0, n-1};
  STAPL_TEST_ONE_OF_REPORT(least_important.descriptor(), ends,
    "Lowest centrality");

  return EXIT_SUCCESS;
}

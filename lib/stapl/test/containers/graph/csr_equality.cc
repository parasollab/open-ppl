/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/containers/graph/csr_graph.hpp>
#include <stapl/containers/graph/short_csr_graph.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>

#include "test_util.h"
#include "../../test_report.hpp"

using namespace stapl;

template<typename Graph, typename CSRGraph>
void test_mesh(std::size_t x, std::size_t y)
{
  auto csr_g = generators::make_mesh<graph_view<CSRGraph>>(x, y);
  auto g = generators::make_mesh<graph_view<Graph>>(x, y);

  // All locations try do add a duplicate edge. Checking to see
  // if the non-multiedges flag appropriately handles this
  csr_g.add_edge_async({0,1});
  rmi_fence();

  csr_g.container().commit();

  bool passed = compare_graphs(csr_g, g);

  STAPL_TEST_REPORT(passed, "CSR mesh equality");
}


template<typename Graph, typename CSRGraph>
void test_edge_list(std::string const& filename)
{
  auto csr_g = read_edge_list<CSRGraph>(filename);
  auto g = read_edge_list<Graph>(filename);

  csr_g.container().commit();

  bool passed = compare_graphs(csr_g, g);

  STAPL_TEST_REPORT(passed, "CSR edge list equality");
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 4) {
    std::cerr << "usage: exe x y edge_list_file" << std::endl;
    exit(1);
  }

  const std::size_t x = atol(argv[1]);
  const std::size_t y = atol(argv[2]);
  const std::string filename = argv[3];

  using csr_graph_type =
    stapl::csr_graph<stapl::DIRECTED, stapl::NONMULTIEDGES>;

  using short_csr_graph_type =
    stapl::short_csr_graph<stapl::DIRECTED>;

  using graph_type = stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES>;

  test_mesh<graph_type, csr_graph_type>(x, y);
  test_mesh<graph_type, short_csr_graph_type>(x, y);
  test_edge_list<graph_type, csr_graph_type>(filename);

  return EXIT_SUCCESS;
}

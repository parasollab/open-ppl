/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <stapl/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/csr_graph.hpp>
#include <stapl/containers/graph/short_csr_graph.hpp>
#include <stapl/containers/graph/out_of_core_graph.hpp>
#include <stapl/containers/graph/generators/binary_tree_network.hpp>

#include "test_util.h"
#include "../../test_report.hpp"


using namespace stapl;

template<typename Graph>
void test_graph(size_t n_levels)
{
  using view_type = stapl::graph_view<Graph>;
  view_type g = stapl::generators::make_binary_tree_network<
    view_type>(n_levels);

  size_t prev_edges = g.num_edges_collective();

  stapl::do_once([&]{
    g.add_edge(1,1);
  });

  one_print("testing first loopback edge\t");
  one_print(g.num_edges_collective() == prev_edges + 1);
  one_print("testing first loopback edge collective");
  one_print(g.num_edges() == prev_edges + 1);

  stapl::do_once([&]{
    g.add_edge(2,2);
  });

  one_print("testing second loopback edge\t");
  one_print(g.num_edges_collective() == prev_edges + 2);
  one_print("testing second loopback edge collective");
  one_print(g.num_edges() == prev_edges + 2);

  rmi_fence();
}


template<typename Graph>
void test_csr_graph(size_t n_levels)
{
  Graph g(9);

  stapl::do_once([&]{
    g.add_edge(1,2);
    g.add_edge(3,2);
  });
  size_t prev_edges = 2;

  stapl::do_once([&]{
    g.add_edge(1,1);
    g.add_edge(2,2);
  });

  one_print("before commit num_edges\t\t");
  one_print(g.num_edges_collective() == prev_edges + 2);
  one_print("before commit num_edges_collective");
  one_print(g.num_edges() == prev_edges + 2);

  g.commit();
  rmi_fence();

  one_print("after commit num_edges\t\t");
  one_print(g.num_edges_collective() == prev_edges + 2);
  one_print("after commit num_edges_collective");
  one_print(g.num_edges() == prev_edges + 2);

  rmi_fence();
}

stapl::exit_code stapl_main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "usage: exe n_levels" << std::endl;
    exit(1);
  }
  srand(1000*stapl::get_location_id() + time(NULL));
  size_t n_levels = boost::lexical_cast<size_t>(argv[1]);

//*
  one_print("***Testing DIRECTED NONMULTIEDGES Dynamic Graph***\n");
  test_graph<stapl::dynamic_graph<stapl::DIRECTED,
             stapl::NONMULTIEDGES,
             size_t, int> >(n_levels);
  one_print("***Testing UNDIRECTED NONMULTIEDGES Dynamic Graph***\n");
  test_graph<stapl::dynamic_graph<stapl::UNDIRECTED,
             stapl::NONMULTIEDGES,
             size_t, int> >(n_levels);
  one_print("***Testing DIRECTED MULTIEDGES Dynamic Graph***\n");
  test_graph<stapl::dynamic_graph<stapl::DIRECTED,
             stapl::MULTIEDGES,
             size_t, int> >(n_levels);
  one_print("***Testing UNDIRECTED MULTIEDGES Dynamic Graph***\n");
  test_graph<stapl::dynamic_graph<stapl::UNDIRECTED,
             stapl::MULTIEDGES,
             size_t, int> >(n_levels);

// */

///*
  one_print("***Testing DIRECTED NONMULTIEDGES csr Graph***\n");
  test_csr_graph<stapl::csr_graph<stapl::DIRECTED,
             stapl::NONMULTIEDGES,
             size_t, int> >(n_levels);
  one_print("***Testing UNDIRECTED NONMULTIEDGES csr Graph***\n");
  test_csr_graph<stapl::csr_graph<stapl::UNDIRECTED,
             stapl::NONMULTIEDGES,
             size_t, int> >(n_levels);
  one_print("***Testing DIRECTED MULTIEDGES csr Graph***\n");
  test_csr_graph<stapl::csr_graph<stapl::DIRECTED,
             stapl::MULTIEDGES,
             size_t, int> >(n_levels);
  one_print("***Testing UNDIRECTED MULTIEDGES csr Graph***\n");
  test_csr_graph<stapl::csr_graph<stapl::UNDIRECTED,
             stapl::MULTIEDGES,
             size_t, int> >(n_levels);

  one_print("***Testing DIRECTED NONMULTIEDGES short csr Graph***\n");
  test_csr_graph<stapl::short_csr_graph<stapl::DIRECTED,
             size_t, int> >(n_levels);
  one_print("***Testing UNDIRECTED NONMULTIEDGES short csr Graph***\n");
  test_csr_graph<stapl::short_csr_graph<stapl::UNDIRECTED,
             size_t, int> >(n_levels);
// */

  return EXIT_SUCCESS;
}

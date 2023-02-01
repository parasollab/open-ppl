/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/views/metadata/projection/geometry.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/domains/indexed.hpp>

#define REPORT_WITH_COLOR
#include "../../../test_report.hpp"

using namespace stapl;

template<typename Graph>
bool edge_exists(Graph& g, std::size_t s, std::size_t t)
{
  typename Graph::edge_descriptor ed(s, t);
  typename Graph::vertex_iterator vit;
  typename Graph::adj_edge_iterator eit;

  return g.find_edge(ed, vit, eit);
}


void two_dimensions()
{
  typedef indexed_domain<std::size_t, 2> domain_type;
  typedef sequential::graph<UNDIRECTED, NONMULTIEDGES, std::string> graph_type;
  typedef geometry_impl::grid_generator<2> generator_type;

  domain_type d(make_tuple(0,0), make_tuple(3, 7));

  generator_type gen(d.dimensions());
  auto grid = gen.build<graph_type>();


  //     0 1 2 3
  //    ---------
  // 0  | | | | |
  //    |-|-|-|-|
  // 1  | | | | |
  //    |-|-|-|-|
  // 2  | | | | |
  //    |-|-|-|-|
  // 3  | | | | |
  //    ---------
  STAPL_TEST_REPORT(grid.get_num_vertices() == d.size(), "Size of grid");
  STAPL_TEST_REPORT(edge_exists(grid, 0, 1), "Edge from 0 to 1");
  STAPL_TEST_REPORT(!edge_exists(grid, 0, 0), "Edge from 0 to 0");
  STAPL_TEST_REPORT(!edge_exists(grid, 0, 2), "Edge from 0 to 2");
}


void three_dimensions()
{
  typedef indexed_domain<std::size_t, 3> domain_type;
  typedef sequential::graph<UNDIRECTED, NONMULTIEDGES, std::string> graph_type;
  typedef geometry_impl::grid_generator<3> generator_type;

  //      -------
  //   1 /--/--/|
  //  0 /  /  / |
  //    -----/|/|
  // 0  | | |/| |
  //    |-|-| |/
  // 1  | | | /
  //    -----/

  domain_type d(make_tuple(0,0,0), make_tuple(1,1,1));

  generator_type gen(d.dimensions());
  auto grid = gen.build<graph_type>();

  STAPL_TEST_REPORT(grid.get_num_vertices() == d.size(), "Size of grid");
  STAPL_TEST_REPORT(edge_exists(grid, 0, 1), "Edge from 0 to 1");
  STAPL_TEST_REPORT(edge_exists(grid, 0, 2), "Edge from 0 to 2");
  STAPL_TEST_REPORT(edge_exists(grid, 0, 4), "Edge from 0 to 4");
  STAPL_TEST_REPORT(edge_exists(grid, 5, 7), "Edge from 5 to 7");
  STAPL_TEST_REPORT(edge_exists(grid, 5, 4), "Edge from 5 to 4");
  STAPL_TEST_REPORT(edge_exists(grid, 5, 1), "Edge from 5 to 1");
  STAPL_TEST_REPORT(!edge_exists(grid, 0, 0), "Edge from 0 to 0");
  STAPL_TEST_REPORT(!edge_exists(grid, 0, 5), "Edge from 0 to 5");
  STAPL_TEST_REPORT(!edge_exists(grid, 6, 1), "Edge from 6 to 1");

}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  two_dimensions();
  three_dimensions();

  return EXIT_SUCCESS;
}

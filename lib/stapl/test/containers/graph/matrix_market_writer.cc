/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/utility/do_once.hpp>
#include <stapl/graph.hpp>


stapl::exit_code stapl_main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cerr << "usage exe input_name output_name\n";
    exit(1);
  }

  stapl::do_once([&] {
    std::cout << "Testing matrix_market_writer...\n";
  });

  using graph_type = stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES, int>;

  std::string input_name = argv[1];
  std::string output_name = argv[2];

  stapl::graph_view< graph_type > my_graph = stapl::read_matrix_market
                                               <graph_type>(input_name, 4096);

  std::string comments = "% 8x8 grid";

  stapl::write_matrix_market(my_graph, output_name, comments);


  return EXIT_SUCCESS;
}

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>


#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/breadth_first_search.hpp>

#include "test_util.h"
#include "algorithm_wrappers.hpp"
#include "test_perf_util.h"
#include "test_algo_perf.h"

using namespace stapl;

template<stapl::graph_attributes Directedness>
void test_part2(std::string alg_name, std::string filename, size_t niter,
          size_t k_start, size_t k_end, int argc, char** argv);

template<stapl::graph_attributes Directedness>
void test_part3(std::string alg_name, std::string filename, size_t niter,
          size_t k_start, size_t k_end, int argc, char** argv);

template<stapl::graph_attributes Directedness>
void test(std::string alg_name, std::string filename, size_t niter,
          size_t k_start, size_t k_end, int argc, char** argv)
{
  if (algorithm_type()(alg_name) < 9)
    test_part2<Directedness>(alg_name, filename, niter, k_start, k_end, argc, argv);
  else
    test_part3<Directedness>(alg_name, filename, niter, k_start, k_end, argc, argv);
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 2) {
    if (stapl::get_location_id() == 0 )
      std::cerr << "usage: " << argv[0] << " [--filename input-filename] "
        << " [--algorithm "
        << "{mssp, topsort, scc, coloring}]"
        << " [--undirected] [--miniterations nexp]"
        << " [--kstart a] [--kend b]"
        << "\n[for generated DAGS in Topological Sort : --dag_size n] "
        << "\n[for Coloring : --torus_size {x y}] "
        << "\n[for SCC : --scc_type {pscc, pscc_single, pscc_schudy}] "
        << std::endl;
    return EXIT_SUCCESS;
  }
  std::string filename  ="";
  std::string alg_name = "mssp";
  bool directed = true;
  size_t niter  = 10;
  size_t k_start = 0;
  size_t k_end = 32;
  srand(0);

  if (stapl::get_location_id() == 0)
    std::cout << "Num locations: " << stapl::get_num_locations() << std::endl;

  for (int i = 1; i < argc; i++) {
    if (!strcmp("--file", argv[i]))
      filename = argv[i+1];
    if (!strcmp("--miniterations", argv[i]))
      niter = atoi(argv[i+1]);
    if (!strcmp("--algorithm", argv[i]))
      alg_name = argv[i+1];
    if (!strcmp("--undirected", argv[i]))
      directed = false;
    if (!strcmp("--kstart", argv[i]))
      k_start = atoi(argv[i+1]);
    if (!strcmp("--kend", argv[i]))
      k_end = atoi(argv[i+1]);
  }

  //Some algorithms needs DIRECTED graphs and don't run with KLA
  if (!strcmp(alg_name.c_str(), "topsort")) {
    if ((!directed) && (stapl::get_location_id() == 0))
      std::cerr << "Topological sort needs Directed graphs"<< std::endl;
    directed = true;
  }
  else if (!strcmp(alg_name.c_str(), "scc")) {
    if ((!directed) && (stapl::get_location_id() == 0))
      std::cerr << "SCC needs Directed graphs"<< std::endl;
    directed = true;
    // SCC is completely sync
    k_start = 0;
    k_end = 0;
  }
  else if (!strcmp(alg_name.c_str(), "coloring")) {
    // Coloring is completely async
    k_start = 0;
    k_end = 0;
  }


  if (directed)
  test<DIRECTED>(alg_name, filename, niter, k_start, k_end, argc, argv);
  else
  test<UNDIRECTED>(alg_name, filename, niter, k_start, k_end, argc, argv);

  return EXIT_SUCCESS;
}

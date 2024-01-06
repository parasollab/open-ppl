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
          size_t k_start, size_t k_end, int argc, char** argv)
{
  switch(algorithm_type()(alg_name)) {
  case 6:
    test_graph<mssp_algo_wrapper<Directedness>, no_generator>(
      filename, niter,
      k_start, k_end,
      argc, argv);
    break;
  case 7:
  {
    size_t n = 1000;
    for (int i = 1; i < argc; i++) {
      if (!strcmp("--dag_size", argv[i])) {
        n = atoi(argv[i+1]);
      }
    }

    dag_generator dag_g(n);
    test_graph<topsort_algo_wrapper<Directedness>, dag_generator>(
      filename, niter,
      k_start, k_end,
      argc, argv,
      dag_g);
    break;
  }
  case 8:
  {
    size_t numx = 100;
    size_t numy = 100;
    for (int i = 1; i < argc; i++) {
      if (!strcmp("--torus_size", argv[i])) {
        numx = atoi(argv[i+1]);
        numy = atoi(argv[i+2]);
      }
    }

    torus_generator torus_g(numx, numy);
    test_graph<coloring_algo_wrapper<Directedness>, torus_generator>(
      filename, niter,
      k_start, k_end,
      argc, argv,
      torus_g);
    break;
  }
  default:
    if (get_location_id() == 0)
    std::cout << "ERROR: Unknown Algorithm Type." << std::endl;
    break;
  }
}

template void test_part2<stapl::DIRECTED>(
  std::string alg_name,
  std::string filename,
  size_t niter,
  size_t k_start, size_t k_end,
  int argc, char** argv);

template void test_part2<stapl::UNDIRECTED>(
  std::string alg_name,
  std::string filename,
  size_t niter,
  size_t k_start, size_t k_end,
  int argc, char** argv);

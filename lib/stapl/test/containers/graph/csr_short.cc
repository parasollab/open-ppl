/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/csr_graph.hpp>
#include <stapl/containers/graph/short_csr_graph.hpp>

#include "test_util.h"

using namespace stapl;

void test_normal_csr()
{
  using graph_t = csr_graph<DIRECTED, MULTIEDGES>;
  using edge_type = graph_t::value_type::adj_edges_type::value_type;

  constexpr std::size_t edge_size = sizeof(edge_type);

  static_assert(edge_size == 24, "Normal edge is 24 bytes");
}

template<typename CSRGraph, int Size>
void test_short_csr()
{
  using vertex_type = typename CSRGraph::value_type;
  using descriptor_type = typename CSRGraph::vertex_descriptor;
  using edge_type = typename vertex_type::adj_edges_type::value_type;

  constexpr std::size_t edge_size = sizeof(edge_type);
  constexpr std::size_t descriptor_size = sizeof(descriptor_type);

  static_assert(edge_size == Size, "Short edge is 4/8 bytes");
  static_assert(descriptor_size == Size, "Descriptor is 4/8 bytes");
}

stapl::exit_code stapl_main(int argc, char* argv[])
{
  test_normal_csr();
  test_short_csr<short_csr_graph<DIRECTED>, 8>();
  test_short_csr<small_short_csr_graph<DIRECTED>, 4>();

  return EXIT_SUCCESS;
}

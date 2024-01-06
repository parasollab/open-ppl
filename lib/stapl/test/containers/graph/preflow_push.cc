/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/containers/graph/algorithms/preflowpush.hpp>

#include "../../test_report.hpp"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Work function to set the capacities of each edge in the graph
///        to a single value.
//////////////////////////////////////////////////////////////////////
class set_capacity
{
  double m_capacity;

public:
  using result_type = void;

  set_capacity(double capacity)
    : m_capacity(capacity)
  { }

  template<typename Vertex>
  result_type operator()(Vertex&& v) const
  {
    for (auto&& e : v)
      e.property().rc(m_capacity);
  }

  void define_type(typer& t)
  {
    t.member(m_capacity);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Set the capacity of all edges to the same value and test if
///        the max flow through the network is a specific value.
///
/// @param x Number of vertices in x-dimension
/// @param y Number of vertices in y-dimension
/// @param k KLA level of asynchrony
/// @param capacity The value to set as the capacity for all edges
/// @param expected_flow The value of flow to test for
//////////////////////////////////////////////////////////////////////
void test_homogeneous_capacity(std::size_t x, std::size_t y, std::size_t k,
                               double capacity, double expected_flow)
{
  using graph_type = graph<DIRECTED, NONMULTIEDGES,
    properties::preflowpush_vertex_property,
    properties::preflowpush_edge_property
  >;

  using view_type = graph_view<graph_type>;

  // Make a directed mesh with 0 as the source and n-1 as the sink
  auto g = generators::make_mesh<view_type>(x, y, false);

  stapl::map_func(set_capacity{capacity}, g);

  auto result = stapl::preflow_push(g, 0, g.num_vertices()-1, k);

  const double flow = get<3>(result);
  const bool is_same = std::fabs(flow - expected_flow) < 0.0001;

  std::ostringstream ss;
  ss << "Computed flow is same as expected (computed = "
     << flow << ", expected = " << expected_flow << ")";

  STAPL_TEST_REPORT(is_same, ss.str());
}


stapl::exit_code stapl_main(int argc, char** argv)
{
  if (argc < 4)
  {
    std::cout <<"usage: " << argv[0] << " x y k" << std::endl;
    exit(1);
  }

  const size_t x = atoi(argv[1]);
  const size_t y = atoi(argv[2]);
  const size_t k = atoi(argv[3]);

  // The flow should be 2, as the sink has two incoming edges with capacity 1
  // and the source has two outgoing capacity 1 edges.
  test_homogeneous_capacity(x, y, k, 1, 2);

  // When all edges have capacity 0, the total flow through the network
  // should also be 0
  test_homogeneous_capacity(x, y, k, 0, 0);

  // The flow should be twice the capacity, for any value x
  test_homogeneous_capacity(x, y, k, x, 2*x);

  return EXIT_SUCCESS;
}

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
#include <stapl/containers/graph/algorithms/triangle_count.hpp>
#include <stapl/containers/graph/algorithms/undirected_triangle_count.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>

#include "../../test_report.hpp"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Add some triangles to the mesh by placing edges across the diagonal
/// @return A pair representing the original number of edges and the number of
///         triangles added
//////////////////////////////////////////////////////////////////////
template<typename View>
std::pair<std::size_t, std::size_t>
add_triangles(View vw, std::size_t num_triangles, std::size_t nx,
              std::size_t ny)
{
  const size_t num_edges_mesh = (((nx-1)*(ny-1)*2) + (nx-1) + (ny-1))*2;

  do_once([&](void) {
    size_t size = vw.size();
    size_t j = 0;
    for (size_t i = 0; i < num_triangles/2; ++i) {
      size_t vd = j++;
      if (vd % nx == nx-1)
        vd = j++;
      size_t tgt = vd + nx + 1;
      if (tgt < size) {
        vw.add_edge_async(vd, tgt);
        vw.add_edge_async(tgt, vd);
      }
    }
  });

  rmi_fence();  // Needed for add_edge_async in do_once.

  const size_t extra_edges = (vw.num_edges_collective() - num_edges_mesh);

  return {num_edges_mesh, extra_edges};
}

//////////////////////////////////////////////////////////////////////
/// @brief Test the triangle_count algorithm.
//////////////////////////////////////////////////////////////////////
void test_triangle_count(std::size_t nx, std::size_t ny,
                         std::size_t num_triangles)
{
  using view_type =
    graph_view<multidigraph<properties::triangle_count_property>>;

  auto vw = generators::make_mesh<view_type>(nx, ny, true);

  // There should be no triangles in a mesh.
  auto triangle_info_0 = stapl::triangle_count(vw, 0);

  STAPL_TEST_REPORT(triangle_info_0.first == 0, "Zero triangles in mesh.");

  auto edge_info = add_triangles(vw, num_triangles, nx, ny);

  auto triangle_info = stapl::triangle_count(vw, 0);
  std::size_t counted = triangle_info.first;

  STAPL_TEST_REPORT(counted == edge_info.second,
    "Correct number of triangles in augmented mesh.");

  // This r should approach 4, the larger the mesh gets, and
  // should always be greater than 1, except in the base-case (no triangles).
  double r = double(3*counted/triangle_info.second - 9*counted)
           / edge_info.first;

  STAPL_TEST_REPORT(r > 1 && r <= 4 && triangle_info_0.second == 0,
    "Clustering coefficient");
}


//////////////////////////////////////////////////////////////////////
/// @brief Test the undirected_triangle_count algorithm.
//////////////////////////////////////////////////////////////////////
void test_undirected_triangle_count(std::size_t nx, std::size_t ny,
                                    std::size_t num_triangles)
{
  using view_type = graph_view<graph<DIRECTED, NONMULTIEDGES, std::size_t>>;

  auto vw = generators::make_mesh<view_type>(nx, ny, true);

  // There should be no triangles in a mesh.
  std::size_t triangle_info_0 = stapl::undirected_triangle_count(vw);

  STAPL_TEST_REPORT(triangle_info_0 == 0, "Zero triangles in mesh.");

  auto edge_info = add_triangles(vw, num_triangles, nx, ny);

  std::size_t counted = stapl::undirected_triangle_count(vw);

  STAPL_TEST_REPORT(counted == edge_info.second,
    "Correct number of triangles in augmented mesh.");
}


stapl::exit_code stapl_main(int argc, char** argv)
{
  size_t nx, ny, num_triangles = 2;

  if (argc > 2) {
    nx = atol(argv[1]);
    ny = atol(argv[2]);
  } else {
    std::cout << "usage: exe x-dim y-dim\n";
    return EXIT_FAILURE;
  }

  for (int i = 1; i < argc; i++) {
    if (!strcmp("--num_triangles", argv[i]))
      num_triangles = atoi(argv[i+1]);
  }

  test_triangle_count(nx, ny, num_triangles);
  test_undirected_triangle_count(nx, ny, num_triangles);

  return EXIT_SUCCESS;
}

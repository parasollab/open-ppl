/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#include <benchmarks/lonestar/delaunay/triangulation/delaunay_triangulation.hpp>

using namespace stapl;
using namespace stapl::delaunay;


stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t num_points = 100;
  size_t seed = 0;
  std::string output_filename;
  bool output = false;
  for (int i = 1; i < argc; i++) {
    if (!strcmp("--num_points", argv[i])) {
      num_points = atoi(argv[i+1]);
    } else if (!strcmp("--output", argv[i])) {
      output = true;
      output_filename = std::string(argv[i+1]);
    } else if (!strcmp("--seed", argv[i])) {
      seed = atoi(argv[i+1]);
    }
  }

  size_t num_locations = get_num_locations();
  do_once([&]() {
    std::cout << "Num Points: " << num_points << std::endl;
    std::cout << " Locations: " << num_locations << std::endl;
    std::cout << "      Seed: " << seed << std::endl;
    std::cout << std::endl;

    std::cout << "Starting..." << std::endl;
  });

  do_once([&]() { std::cout << "Testing with static graph..." << std::endl; });
  delaunay_triangulation<graph> dt1;
  dt1.triangulate(num_points, seed);

  typedef typename delaunay_triangulation<graph>::view_type view_type;

  if (output) {
    write_edge_list<view_type>(dt1.m_mesh_view, output_filename + ".out");
    dt1.m_mesh_view.write_dot(output_filename + ".dot");
  }
  rmi_fence();

  return EXIT_SUCCESS;
}


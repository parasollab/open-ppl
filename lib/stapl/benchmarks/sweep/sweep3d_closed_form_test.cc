/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <benchmarks/sweep/sweep.hpp>
#include <benchmarks/sweep/sweep_property.hpp>
#include <benchmarks/sweep/sweep_utils.hpp>
#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/generators/grid.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/utility/do_once.hpp>
#include <test/containers/graph/test_util.h>
#include <sstream>

using namespace std;
using namespace stapl::sweep_user_wf;
using namespace stapl::sweep_utils;

size_t TUNING = 2;
size_t nx, ny, nz;

stapl::exit_code stapl_main(int argc,char** argv)
{
  if (argc > 3) {
    nx = atol(argv[1]);
    ny = atol(argv[2]);
    nz = atol(argv[3]);
  } else {
    cout<<"usage: exe x-dim y-dim z-dim [--tuning k]\n";
    return EXIT_FAILURE;
  }

  srand(0);

  for (int i = 3; i < argc; i++) {
    if (!strcmp("--tuning", argv[i]))
      TUNING = atoi(argv[i+1]);
  }

  typedef std::vector<float> dimensions_t;
  typedef stapl::multidigraph<stapl::properties::sweep_property<dimensions_t>,
                              dimensions_t> PGR_static;
  typedef stapl::graph_view<PGR_static> graph_view_t;
  typedef typename graph_view_t::vertex_descriptor vd_type;
  std::array<size_t, 3> dims({{nx, ny, nz}});
  graph_view_t vw = stapl::generators::make_grid<graph_view_t>(dims);

  stapl::map_func(set_coordinates_wf(nx, ny, true), vw);
  stapl::graph_paradigm(set_edges_wf(), set_edges_update_func(), vw);
  // stapl::map_func(init_edges_3D_wf(nx, ny, nz), vw);

  one_print("Testing SWEEP-3D...\t\t\t\t");

  // directions:
  std::vector<dimensions_t> directions = {{1,1,1}, {1,1,-1}, {1,-1,1},
                                          {1,-1,-1}, {-1,1,1}, {-1,1,-1},
                                          {-1,-1,1}, {-1,-1,-1}};

  // kla SWEEP:
  stapl::counter<stapl::default_timer> time;
  time.start();
  stapl::sweep(vw, directions,
               init_property(),
               update_property(),
               update_condition(),
               filter_edge(),
               TUNING);
  double d = time.stop();

  // stapl::map_func(print_level_wf(nx*ny*nz), vw);
  bool result = stapl::map_reduce(check_level_3D_wf(nx, ny, nz, directions),
                                  stapl::logical_and<bool>(), vw);

  stapl::do_once([d, result]{
      std::string s = result ? "passed" : "failed";
      std::cout << " ["<< s << "]  (time: " << d << " s)" << std::endl;
    });

  return EXIT_SUCCESS;
}

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
#include <stapl/containers/graph/digraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
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
  string fname;
  if (argc > 1) {
    fname = argv[1];
  } else {
    cout<<"usage: exe filename [--tuning k]\n";
    return EXIT_FAILURE;
  }

  srand(0);

  for (int i = 1; i < argc; i++) {
    if (!strcmp("--tuning", argv[i]))
      TUNING = atoi(argv[i+1]);
  }

  typedef std::vector<float> dimensions_t;
  typedef stapl::digraph<stapl::properties::sweep_property<dimensions_t>,
                         dimensions_t> PGR_static;
  typedef stapl::graph_view<PGR_static> graph_view_t;
  typedef typename graph_view_t::vertex_descriptor vd_type;
  graph_view_t vw = sweep_graph_reader<PGR_static>(fname);

  // stapl::map_func(print_graph_wf(), vw);
  one_print("Testing SWEEP...\t\t\t\t");

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

  // stapl::map_func(print_level_wf(nx*ny), vw);
  stapl::do_once([d]{ std::cout << "  (time: " << d << " s)" << std::endl; });

  return EXIT_SUCCESS;
}

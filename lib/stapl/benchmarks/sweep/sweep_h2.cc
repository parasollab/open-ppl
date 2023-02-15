/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <benchmarks/sweep/sweep_h.hpp>
#include <benchmarks/sweep/sweep_property.hpp>
#include <benchmarks/sweep/sweep_utils_h.hpp>
#include <benchmarks/sweep/sweep_h_test_util.hpp>
#include <stapl/containers/array/static_array_fwd.hpp>
#include <stapl/containers/graph/digraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/containers/graph/algorithms/create_level.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/utility/do_once.hpp>
#include <test/containers/graph/test_util.h>
#include <sstream>

using namespace std;
using namespace stapl;
using namespace stapl::sweep_user_wf;
using namespace stapl::sweep_utils;

size_t TUNING = 2;
size_t nx, ny, nz, c;

struct hierarchy_map_wf
{
  typedef void result_type;

  template <class NativeGraph, class NativeArray>
  void operator()(NativeGraph&& g, NativeArray&& a)
  {
    auto leader = (*g.begin()).descriptor();
    for (auto e : a)
      e = leader;
  }
};

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

  for (int i = 4; i < argc; i++) {
    if (!strcmp("--tuning", argv[i]))
      TUNING = atoi(argv[i+1]);
  }

  typedef std::vector<float> dimensions_t;
  typedef stapl::digraph<
    super_vertex_property<stapl::properties::sweep_property<dimensions_t>>,
    super_edge_property<stapl::properties::sweep_edge_property<dimensions_t>>>
    PGR_static;
  typedef stapl::graph_view<PGR_static> graph_view_t;
  typedef typename graph_view_t::vertex_descriptor vd_type;
  one_print("Reading Graph\n");
  graph_view_t vw = sweep_graph_reader<PGR_static>(fname);

  // Create a hierarchy.
  one_print("Creating Hierarchy (Setup)\n");
  typedef static_array<size_t> array_t;
  array_t group_array(vw.size());
  typedef array_view<array_t> array_view_t;
  array_view_t group_array_vw(group_array);

  map_func(hierarchy_map_wf(), native_view(vw), native_view(group_array_vw));

  graph_external_property_map<typename graph_view_t::view_container_type,
                              size_t, array_view_t>
    group_id_prop_map(vw.container(), group_array_vw);

  one_print("Creating Hierarchy\n");
  graph_view_t vw1 = create_level(vw, group_id_prop_map,
                                  vpselect_wf(), epselect_wf());
  rmi_fence();

  one_print("Testing SWEEP...\t\t\t\t");

  std::vector<graph_view_t*> hierarchy{&vw, &vw1};

  // directions:
  std::vector<dimensions_t> directions = {{1,1,1}, {1,1,-1}, {1,-1,1},
                                          {1,-1,-1}, {-1,1,1}, {-1,1,-1},
                                          {-1,-1,1}, {-1,-1,-1}};

  // kla SWEEP:
  stapl::counter<stapl::default_timer> time;
  time.start();
  stapl::sweep(hierarchy,
               directions,
               init_property(),
               update_h_property(),
               update_condition(),
               filter_h_edge(),
               TUNING);
  double d = time.stop();

  stapl::do_once([d]{ std::cout << " (time: " << d << " s)" << std::endl; });

  return EXIT_SUCCESS;
}

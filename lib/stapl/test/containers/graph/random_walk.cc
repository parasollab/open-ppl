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
#include <stapl/containers/graph/algorithms/random_walk.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>

#include "test_util.h"

using namespace std;

stapl::exit_code stapl_main(int argc,char** argv)
{
  size_t nx, ny;
  size_t tuning = 0;
  size_t source = 0;
  size_t path_length = 10;
  bool   print_path = false;

  if (argc > 2) {
    nx = atol(argv[1]);
    ny = atol(argv[2]);
  } else {
    cout<<"usage: exe x-dim y-dim\n";
    return EXIT_FAILURE;
  }

  for (int i = 1; i < argc; i++) {
    if (!strcmp("--tuning", argv[i]))
      tuning = atoi(argv[i+1]);
    if (!strcmp("--source", argv[i]))
      source = atoi(argv[i+1]);
    if (!strcmp("--path_length", argv[i]))
      path_length = atoi(argv[i+1]);
    if (!strcmp("--print_path", argv[i]))
      print_path = true;
  }

  typedef stapl::multidigraph<stapl::properties::rw_property> PGR;
  typedef stapl::graph_view<PGR> graph_view_t;
  graph_view_t vw = stapl::generators::make_mesh<graph_view_t>(nx, ny);

  // perform the walk.
  one_print("Testing Random-Walk...\t\t\t");
  auto x = random_walk(vw, source, path_length, tuning);

  bool levels_passed = true;
  size_t cnt = 1;
  stapl::do_once([&](void) {
      for (auto const& e : x) {

        if (print_path) {
          std::cout << e << " ";
          if (e != std::numeric_limits<size_t>::max())
            std::cout << vw[e].property().level() << " / " << cnt;
          std::cout << endl;
        }

        if (e == std::numeric_limits<size_t>::max() ||
            vw[e].property().level() < cnt)
          levels_passed = false;
        ++cnt;
      }
    });

  one_print(x.size() == path_length && levels_passed);

  return EXIT_SUCCESS;
}

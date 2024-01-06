/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/algorithms/undirected_triangle_count.hpp>

#include "../../common/command_line_app.hpp"
#include "../../common/graph_benchmark.hpp"

stapl::exit_code stapl_main(int argc, char* argv[])
{
  command_line_app<no_options> app;
  app(argc, argv);
  app.supported_paradigms("lsync");

  auto vw = app.create_graph<
    std::size_t, stapl::properties::no_property, graph_model::adj
  >();

  auto bench = benchmark_builder{}
    .with_setup([&]() { return 0; })
    .with_runner([&](int trial, int) {
      return undirected_triangle_count(vw);
    })
    .with_stats([&](std::size_t tris) { return tris; })
    .with_trials(app.trials())
    .build();

  bench(std::cout);

  return EXIT_SUCCESS;
}

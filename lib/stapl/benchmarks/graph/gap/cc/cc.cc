/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/algorithms/connected_components.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>

#include "../../common/command_line_app.hpp"
#include "../../common/graph_benchmark.hpp"
#include "verify.hpp"

namespace po = boost::program_options;

stapl::exit_code stapl_main(int argc, char* argv[])
{
  command_line_app<no_options> app;
  app(argc, argv);

  auto vw = app.create_graph<stapl::properties::cc_property>();
  auto policy = app.execution_policy(vw);

  auto bench = benchmark_builder{}
    .with_setup([&]() { return 0; })
    .with_runner([&](int trial, int) {
      return connected_components(policy, vw);
    })
    .with_stats([&](std::size_t iters) { return iters; })
    .with_verifier([&](int) {
      return verify_cc(vw);
    })
    .with_trials(app.trials())
    .build();

  bench(std::cout);

  return EXIT_SUCCESS;
}

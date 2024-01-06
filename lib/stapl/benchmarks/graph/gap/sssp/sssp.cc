/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/algorithms/sssp.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>

#include "../../common/command_line_app.hpp"
#include "../../common/sources.hpp"
#include "../../common/graph_benchmark.hpp"
#include "verify.hpp"

stapl::exit_code stapl_main(int argc, char* argv[])
{
  command_line_app<no_options> app;
  app(argc, argv);

  const auto trials = app.trials();
  auto vw = app.create_graph<stapl::properties::sssp_property, float>();
  app.supported_paradigms("kla");

  using sources_type = std::vector<std::uint32_t>;

  auto bench = benchmark_builder{}
    .with_setup([&]() -> sources_type {
      return select_sources(vw, trials);
    })
    .with_runner([&](int trial, sources_type const& sources) {
      return sssp(vw, sources[trial], app.k());
    })
    .with_stats([&](std::size_t iters) { return iters; })
    .with_verifier([&](std::size_t iters) {
      return verify_sssp(vw);
    })
    .with_trials(trials)
    .build();

  bench(std::cout);

  return EXIT_SUCCESS;
}

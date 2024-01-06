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
#include <stapl/containers/graph/algorithms/betweenness_centrality.hpp>

#include "../../common/command_line_app.hpp"
#include "../../common/graph_benchmark.hpp"

namespace po = boost::program_options;

class betweenness_options
{
  std::size_t m_num_sources;

public:
  void opts(po::options_description& desc)
  {
    desc.add_options()
      ("num_sources", po::value<std::size_t>()->required(),
       "Number of sources active at a time");
  }

  void parse(po::variables_map const& vars)
  {
    m_num_sources = vars["num_sources"].template as<std::size_t>();
  }

  std::size_t num_sources() const
  {
    return m_num_sources;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  command_line_app<betweenness_options> app;
  app(argc, argv);

  const std::size_t num_sources = app.num_sources();

  auto vw = app.create_graph<stapl::properties::bc_property>();
  auto policy = app.execution_policy(vw);

  auto bench = benchmark_builder{}
    .with_setup([&]() { return 0; })
    .with_runner([&](int trial, int) {
      betweenness_centrality(policy, vw, num_sources, true);
      return 0;
    })
    .with_stats([&](int) { return 0; })
    .with_trials(app.trials())
    .build();

  bench(std::cout);

  return EXIT_SUCCESS;
}

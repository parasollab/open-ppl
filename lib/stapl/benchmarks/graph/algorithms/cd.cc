/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/algorithms/community_detection.hpp>

#include "../common/command_line_app.hpp"
#include "../common/graph_benchmark.hpp"

class community_detection_options
{
private:
  std::size_t m_iter;

public:
  void opts(po::options_description& desc)
  {
    desc.add_options()
      ("iterations", po::value<std::size_t>()->required(),
       "The number of CommunityDetection iteration to perform.");
  }

  void parse(po::variables_map const& vars)
  {
    m_iter = vars["iterations"].template as<std::size_t>();
  }

  std::size_t iterations() const
  {
    return m_iter;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  command_line_app<community_detection_options> app;
  app(argc, argv);

  const auto it = app.iterations();
  const auto trials = app.trials();

  app.supported_paradigms("kla", "async", "lsync", "hier", "hubs");
  auto vw = app.create_graph<stapl::properties::cd_property>();
  auto policy = app.execution_policy(vw);

  auto bench = benchmark_builder{}
    .with_setup([&]() {
      return 0;
    })
    .with_runner([&](int trial, int) {
      return community_detection(policy, vw, it);
    })
    .with_stats([&](std::size_t iter) {
      return iter;
    })
    .with_trials(trials)
    .build();

  bench(std::cout);

  return EXIT_SUCCESS;
}

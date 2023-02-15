/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/algorithms/page_rank.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>

#include "../../common/command_line_app.hpp"
#include "../../common/graph_benchmark.hpp"
#include "verify.hpp"

class page_rank_options
{
  std::size_t m_iterations;
  double m_damping;

public:
  void opts(po::options_description& desc)
  {
    desc.add_options()
      ("iterations", po::value<std::size_t>()->default_value(20),
       "Number of PageRank iterations")
      ("damping", po::value<double>()->default_value(0.85),
       "PageRank damping factor") ;
  }

  void parse(po::variables_map const& vars)
  {
    m_iterations = vars["iterations"].template as<std::size_t>();
    m_damping = vars["damping"].template as<double>();
  }

  std::size_t iterations() const
  {
    return m_iterations;
  }

  double damping() const
  {
    return m_damping;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  command_line_app<page_rank_options> app;
  app(argc, argv);

  const std::size_t iterations = app.iterations();
  const double damping = app.damping();

  auto vw = app.create_graph<stapl::properties::page_rank_property<float>>();
  auto policy = app.execution_policy(vw);

  auto bench = benchmark_builder{}
    .with_setup([&]() { return 0; })
    .with_runner([&](int trial, int) {
      return page_rank(policy, vw, iterations, damping);
    })
    .with_stats([&](std::size_t iters) { return iters; })
    .with_verifier([&](std::size_t iters) {
      return verify_page_rank(vw, damping);
    })
    .with_trials(app.trials())
    .build();

  bench(std::cout);

  return EXIT_SUCCESS;
}

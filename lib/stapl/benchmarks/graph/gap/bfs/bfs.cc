/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/breadth_first_search.hpp>
#include <stapl/containers/graph/algorithms/breadth_first_level.hpp>

#include "../../common/command_line_app.hpp"
#include "../../common/sources.hpp"
#include "../../common/graph_benchmark.hpp"
#include "verify.hpp"
#include "verify_level.hpp"
#include <string>


class bfs_options
{
private:
  std::string m_compute;

public:
  void opts(po::options_description& desc)
  {
    desc.add_options()
      ("compute", po::value<std::string>()->default_value("both"),
       "BFS properties to compute (level | parent | both).");
  }

  void parse(po::variables_map const& vars)
  {
    m_compute = vars["compute"].template as<std::string>();
  }

  std::string compute() const
  {
    return m_compute;
  }
};


void run_breadth_first_search(const command_line_app<bfs_options>& app)
{
  auto trials = app.trials();
  auto vw = app.create_graph<stapl::properties::bfs_property>();
  auto policy = app.execution_policy(vw);

  using sources_type = std::vector<std::uint32_t>;

  auto bench = benchmark_builder{}
    .with_setup([&]() -> sources_type {
      return select_sources(vw, trials);
    })
    .with_runner([&](int trial, sources_type const& sources) {
      return breadth_first_search(policy, vw, sources[trial]);
    })
    .with_stats([&](std::size_t iters) { return iters; })
    .with_verifier([&](std::size_t iters) {
      return verify_bfs(vw);
    })
    .with_trials(trials)
    .build();
  bench(std::cout);
}


void run_breadth_first_level(const command_line_app<bfs_options>& app)
{
  auto trials = app.trials();
  auto vw = app.create_graph<std::uint32_t>();
  auto policy = app.execution_policy(vw);

  using sources_type = std::vector<std::uint32_t>;

  auto bench = benchmark_builder{}
    .with_setup([&]() -> sources_type {
      return select_sources(vw, trials);
    })
    .with_runner([&](int trial, sources_type const& sources) {
      return breadth_first_level(policy, vw, sources[trial]);
    })
    .with_stats([&](std::size_t iters) { return iters; })
    .with_verifier([&](std::size_t iters) {
      return verify_level(vw);
    })
    .with_trials(trials)
    .build();
  bench(std::cout);
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  command_line_app<bfs_options> app;
  app(argc, argv);

  auto compute = app.compute();

  if (compute == "level")
    run_breadth_first_level(app);
  else if (compute == "both")
    run_breadth_first_search(app);
  else
    stapl::do_once([&](){
        std::cout << "ERROR! BFS version not supported.\n";
    });

  return EXIT_SUCCESS;
}

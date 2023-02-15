/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/k_core.hpp>

#include "../common/command_line_app.hpp"
#include "../common/graph_benchmark.hpp"

class kc_options
{
private:
  int m_core_sz;

public:
  void opts(po::options_description& desc)
  {
    desc.add_options()
      ("core_sz", po::value<int>()->required(),
       "Parameter to control which vertices are deleted.");
  }

  void parse(po::variables_map const& vars)
  {
    m_core_sz = vars["core_sz"].template as<int>();
  }

  int core_sz() const
  {
    return m_core_sz;
  }

};

/////////////////////////////////////////////////////////////////
///@brief Checks if the number of edges are more than core size
///       if they aren't already deleted.
/////////////////////////////////////////////////////////////////
struct verify_edges
{
  int m_core_sz;

  verify_edges(int core_sz)
    : m_core_sz(core_sz)
  {}

  template<typename Vertex>
  bool operator()(Vertex&& v)
  {
    return (v.property() == -1 || v.property() >= m_core_sz);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_core_sz);
  }
};


/////////////////////////////////////////////////////////////////
///@brief Returns 1 if the vertex exists and 0 otheriwse.
/////////////////////////////////////////////////////////////////
struct num_vertices
{
  template<typename Vertex>
  std::size_t operator()(Vertex&& v)
  {
    if (v.property() != -1)
      return 1;
    return 0;
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  command_line_app<kc_options> app;
  app(argc, argv);

  const auto core_sz = app.core_sz();
  const auto trials = app.trials();

  auto vw = app.create_graph<int>();
  auto policy = app.execution_policy(vw);
  app.supported_paradigms("lsync", "kla", "async", "hier", "hubs");

  auto bench = benchmark_builder{}
    .with_setup([&]() {
      return 0;
    })
    .with_runner([&](int trial, int) {
      return k_core(policy, vw, core_sz);
    })
    .with_stats([&](std::size_t iter) {
      return stapl::map_reduce(num_vertices{}, stapl::plus<std::size_t>(), vw);
    })
    .with_verifier([&](std::size_t) {
      return stapl::map_reduce(verify_edges{core_sz},
                               stapl::logical_and<bool>(), vw);
    })
    .with_trials(trials)
    .build();

  bench(std::cout);

  return EXIT_SUCCESS;
}

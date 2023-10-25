/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/algorithms/graph_io.hpp>
#include <stapl/containers/graph/algorithms/bad_rank.hpp>
#include <random>

#include "../common/command_line_app.hpp"
#include "../common/graph_benchmark.hpp"

class bad_rank_options
{
private:
  std::size_t m_iter;
  double m_prob_bl;
  double m_damping;

public:
  void opts(po::options_description& desc)
  {
    desc.add_options()
      ("blacklist_prob", po::value<double>()->required(),
              "Probibility of blacklisted vertices in the graph.")
      ("damping", po::value<double>()->required(),
              "Damping factor for the algorithm.")
      ("iterations", po::value<std::size_t>()->required(),
              "The number of badrank iteration to perform.");

  }

  void parse(po::variables_map const& vars)
  {
    m_prob_bl = vars["blacklist_prob"].template as<double>();
    m_damping = vars["damping"].template as<double>();
    m_iter = vars["iterations"].template as<std::size_t>();
  }

  double prob() const
  {
    return m_prob_bl;
  }

  double dam() const
  {
    return m_damping;
  }

  std::size_t it() const
  {
    return m_iter;
  }
};

struct br_wf
{

  double m_prob;
  std::mt19937 m_gen;

  br_wf(double p)
    : m_prob(p),  m_gen(std::random_device()())
  {}

  template<typename V>
  std::size_t operator()(V&& v)
  {
    std::bernoulli_distribution bern(m_prob);

    if (bern(m_gen))
    {
      v.property().set_blacklisted(true);
      return 1;
    }
    return 0;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_prob);
  }
};

template<bool Blacklisted>
struct bl_wf
{

  template<typename V>
  std::size_t operator()(V&& v)
  {
    const bool val = Blacklisted ?
      v.property().is_blacklisted() : !v.property().is_blacklisted();
    return val ? v.property().rank() : 0;
  }

};

stapl::exit_code stapl_main(int argc, char* argv[])
{
  command_line_app<bad_rank_options> app;
  app(argc, argv);

  const auto prob = app.prob();
  const auto dam = app.dam();
  const auto it = app.it();
  const auto trials = app.trials();

  auto vw = app.create_graph<stapl::properties::bad_rank_property>();
  auto policy = app.execution_policy(vw);
  app.supported_paradigms("lsync", "hier", "hubs");
  auto num_bl = stapl::map_reduce(br_wf{prob}, stapl::plus<std::size_t>{}, vw);

  auto avg_bl_rank =
    stapl::map_reduce(bl_wf<true>{}, stapl::plus<double>{}, vw)/num_bl;

  auto num_nbl = vw.size() - num_bl;

  auto avg_nbl_rank =
    stapl::map_reduce(bl_wf<false>{}, stapl::plus<double>{}, vw)/num_nbl;

  using view_type = decltype(vw);

  auto bench = benchmark_builder{}
    .with_setup([&]() {
      return 0;
    })
    .with_runner([&](int trial, int) {
      return bad_rank(policy, vw, it, num_bl, dam);
    })
    .with_stats([&](std::size_t) {
      return num_bl;
    })
    .with_verifier([&](std::size_t) {
      return avg_bl_rank < avg_nbl_rank ? false : true;
    })
    .with_trials(trials)
    .build();

  bench(std::cout);

  return EXIT_SUCCESS;
}

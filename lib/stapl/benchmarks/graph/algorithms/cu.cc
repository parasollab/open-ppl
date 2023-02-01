/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/algorithms/cut_conductance.hpp>
#include <random>

#include "../common/command_line_app.hpp"
#include "../common/graph_benchmark.hpp"

class cu_options
{
private:
  std::size_t m_id;
  std::size_t m_sets;

public:
  static void opts(po::options_description& desc)
  {
    desc.add_options()
      ("id", po::value<std::size_t>()->required(),
       "The set id to compute conductance for")
      ("sets", po::value<std::size_t>()->required(),
       "Number of sets.");
  }

  void parse(po::variables_map const& vars)
  {
    m_id = vars["id"].template as<std::size_t>();
    m_sets = vars["sets"].template as<std::size_t>();
  }

  std::size_t id() const
  {
    return m_id;
  }

  std::size_t sets() const
  {
    return m_sets;
  }
};


////////////////////////////////////////////////////////////////////
/// @brief Work-function to assign random set id's to vertices, using
///        uniform dist.
////////////////////////////////////////////////////////////////////
struct random_set_id
{
  std::size_t m_sets;
  std::mt19937 m_gen;

  random_set_id(std::size_t s)
    : m_sets(s), m_gen(std::random_device()())
  {}

  template<typename V>
  std::size_t operator()(V&& v)
  {
    std::uniform_int_distribution<> dist(0, m_sets-1);
    return dist(m_gen);
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_sets);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  command_line_app<cu_options> app;
  app(argc, argv);

  const auto trials = app.trials();
  std::size_t id = app.id();
  std::size_t num_set = app.sets();

  auto vw = app.create_graph<stapl::properties::no_property>();
  app.supported_paradigms("lsync");

  using view_type = decltype(vw);
  using member_storage_type = stapl::array<std::size_t>;
  using member_view_type = stapl::array_view<member_storage_type>;
  using member_map_type = stapl::graph_external_property_map<view_type,
        std::size_t, member_view_type>;

  member_storage_type m(vw.size());

  auto bench = benchmark_builder{}
    .with_setup([&](){
      member_view_type members(m);

      stapl::transform(members, members, random_set_id{num_set});

      member_map_type member_map(vw, members);

      return member_map;
    })
    .with_runner([&](int trial, member_map_type const& member_map) {
      return cut_conductance(vw, member_map, id++ % num_set);
    })
    .with_stats([&](double cut) {
      return cut;
    })
    .with_verifier([&](double cut) {
      return (cut >= 0 && cut <= 1);
    })
    .with_trials(trials)
    .build();

  bench(std::cout);

  return EXIT_SUCCESS;
}

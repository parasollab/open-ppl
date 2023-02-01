/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/
#include <stapl/containers/graph/algorithms/link_prediction.hpp>
#include <boost/variant/variant.hpp>
#include <cmath>

#include "../common/command_line_app.hpp"
#include "../common/graph_benchmark.hpp"

struct check_edges
{
  using result_type = bool;

  template<typename V>
  bool operator()(V&& v) const
  {
    for (auto const& e : v)
    {
      if (std::fabs(1.0 - v.property().link_probability(e.target())) > 1E-12 &&
          v.descriptor() != e.target())
        return false;
    }
    return true;
  }
};


/////////////////////////////////////////////////////////////////////////
/// @brief Helper to invoke the correct execution strategy based on the
/// provided policy.
///
/// @tparam View Type of the input @ref graph_view.
/////////////////////////////////////////////////////////////////////////
template<typename View>
struct execute_policy : boost::static_visitor<void>
{
  View& m_vw;
  execute_policy(View& view)
    : m_vw(view)
  {}

  void operator()(stapl::sgl::kla_policy& policy) const
  {
    return link_prediction(m_vw, policy.k());
  }

  void operator()(stapl::sgl::level_sync_policy& policy) const
  {
    return link_prediction(m_vw, 0);
  }

  void operator()(stapl::sgl::async_policy& policy) const
  {
    return link_prediction(m_vw, std::numeric_limits<std::size_t>::max()-1);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  command_line_app<no_options> app;
  app(argc, argv);

  const auto trials = app.trials();

  app.supported_paradigms("kla", "lsync", "async");
  auto vw = app.create_graph<stapl::properties::lp_property<unsigned int>>();
  auto policy = app.execution_policy(vw);

  using view_type = decltype(vw);

  auto bench = benchmark_builder{}
    .with_setup([&](){  return 0; })
    .with_runner([&](int trial, int) {
      boost::apply_visitor(execute_policy<view_type>{vw}, policy);
      return 0;
    })
    .with_stats([&](std::size_t) { return 0; })
    .with_verifier([&](std::size_t) {
      return stapl::map_reduce(check_edges{}, stapl::logical_and<bool>(), vw);
    })
    .with_trials(trials)
    .build();

  bench(std::cout);

  return EXIT_SUCCESS;
}

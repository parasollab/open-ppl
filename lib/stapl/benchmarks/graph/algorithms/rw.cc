/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/algorithms/random_walk.hpp>
#include <boost/variant/variant.hpp>
#include <algorithm>
#include <vector>

#include "../common/command_line_app.hpp"
#include "../common/sources.hpp"
#include "../common/graph_benchmark.hpp"


class rw_options
{
private:
  std::size_t m_path_length;

public:
  void opts(po::options_description& desc)
  {
    desc.add_options()
      ("path_length", po::value<std::size_t>()->required(),
              "Desired path length.");
  }

  void parse(po::variables_map const& vars)
  {
    m_path_length = vars["path_length"].template as<std::size_t>();
  }

  std::size_t path_length() const
  {
    return m_path_length;
  }
};


/////////////////////////////////////////////////////////////////////////
/// @brief Helper to invoke the correct execution strategy based on the
/// provided policy.
///
/// @tparam PView Type of the return_type.
/// @tparam GView Type of the input @ref view.
/// @tparam Source Type of the input @ref source.
/////////////////////////////////////////////////////////////////////////
template<typename PView, typename GView, typename Source>
struct execute_policy
  : boost::static_visitor<PView>
{
  using return_type = PView;
  GView& m_vw;
  Source m_source;
  std::size_t m_path_length;
  int m_trial;

  execute_policy(GView& view, Source const& source,
                 std::size_t path_length, int trial)
    : m_vw(view), m_source(source), m_path_length(path_length), m_trial(trial)
  {}

  return_type operator()(stapl::sgl::kla_policy& policy) const
  {
    return random_walk(m_vw, m_source[m_trial], m_path_length, policy.k());
  }

  return_type operator()(stapl::sgl::level_sync_policy& policy) const
  {
    return random_walk(m_vw, m_source[m_trial], m_path_length, 0);
  }

  return_type operator()(stapl::sgl::async_policy& policy) const
  {
    return random_walk(m_vw, m_source[m_trial], m_path_length, m_vw.size());
  }
};


template<typename View>
struct random_walk_path
{
  View path;
};


template<typename View>
std::ostream& operator<<(std::ostream& os, random_walk_path<View> const& rwp)
{
  for (auto v : rwp.path)
    os << v << " ";
  return os;
}


template<typename View>
bool has_edge(View& view, typename View::vertex_descriptor source,
    typename View::vertex_descriptor target)
{
  for (auto const& e : view[source])
  {
    if (e.target() == target)
      return true;
  }
  return false;
}


template<typename PView, typename GView>
bool check_has_edges(PView const& path_view, GView& view)
{
  std::size_t index = 1;
  for (auto const& v : path_view)
  {
    if (index >= path_view.size())
      break;

    if (!has_edge(view, v, path_view[index++]))
      return false;
  }
  return true;
}


template<typename PView, typename GView>
bool check_level(PView const& path_view, GView& vw)
{
  std::size_t level = 1;
  for (auto const& v : path_view)
  {
    if (vw[v].property().level() != level)
      return false;
    ++level;
  }
  return true;
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  command_line_app<rw_options> app;
  app(argc, argv);

  const std::size_t trials = app.trials();
  const std::size_t path_length = app.path_length();

  app.supported_paradigms("kla", "lsync", "async");
  auto vw = app.create_graph<stapl::properties::rw_property>();

  using view_type = decltype(vw);
  using vd_type = view_type::vertex_descriptor;
  using sources_type = std::vector<vd_type>;
  using path_view_type = array_view<stapl::array<vd_type>>;

  auto policy = app.execution_policy(vw);

  auto bench = benchmark_builder{}
    .with_setup([&]() -> sources_type {
      return select_sources(vw, trials);
    })
    .with_runner([&](int trial, sources_type const& sources) {
      return boost::apply_visitor(execute_policy<path_view_type,
        view_type, sources_type>{vw, sources, path_length, trial}, policy);
    })
    .with_stats([&](path_view_type const& path_view) {
      return random_walk_path<path_view_type>{path_view};
    })
    .with_verifier([&](random_walk_path<path_view_type> const& rwp) {
      return stapl::do_once([&]() {
        std::vector<std::size_t> path(path_length);

        // removes non unique vertices. So we can compare size and
        // identify if there is a loop in the path or not.
        std::copy(rwp.path.begin(), rwp.path.end(), path.begin());
        std::sort(path.begin(), path.end());
        path.erase(std::unique(path.begin(), path.end()), path.end());

        bool all_unique_vertices = path.size() == rwp.path.size();

        bool verify = check_has_edges(rwp.path, vw);

        if (all_unique_vertices)
          verify &= check_level(rwp.path, vw);

        return verify;
      });
    })
    .with_trials(trials)
    .build();

  bench(std::cout);

  return EXIT_SUCCESS;
}

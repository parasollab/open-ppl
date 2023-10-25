/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/breadth_first_search.hpp>
#include <stapl/containers/graph/algorithms/approx_breadth_first_search.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/array.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/containers/graph/generators/erdos_renyi.hpp>

#include "../../test_report.hpp"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Work function to exact the level from a single vertex
//////////////////////////////////////////////////////////////////////
struct extract_level
{
  typedef void result_type;

  template<typename T, typename U>
  void operator() (T v, U e) const
  {
    e = v.property().level();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to compute the error of approximation.
//////////////////////////////////////////////////////////////////////
struct error_wf
{
  using result_type = double;

  template<typename Approximate, typename Exact>
  result_type operator()(Approximate a, Exact e)
  {
    return e > 0 ?
      (static_cast<double>(a) - static_cast<double>(e))
        / static_cast<double>(e) : 0;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to check if an approximate distance makes
///        sense with respect to the exact distance
//////////////////////////////////////////////////////////////////////
struct validate_level
{
  using result_type = bool;

  template<typename Approximate, typename Exact>
  result_type operator()(Approximate a, Exact e)
  {
    // A vertex in real BFS tree was not found
    if (a == 0 && e != 0)
      return false;
    // A vertex that is not in real BFS tree was somehow found.
    else if (e == 0 && a != 0)
      return false;
    // Approximation somehow found a shorter path.
    else if (a < e)
      return false;

    return true;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Create an array of the discovered levels for each vertex
//////////////////////////////////////////////////////////////////////
template<typename GraphView>
array_view<stapl::array<std::size_t>> levels(GraphView const& view)
{
  auto* a = new stapl::array<std::size_t>(view.size());
  array_view<stapl::array<std::size_t>> levels_view(a);
  stapl::map_func(extract_level(), view, levels_view);

  return levels_view;
}

//////////////////////////////////////////////////////////////////////
/// @brief Choose a random starting point for the traversal
//////////////////////////////////////////////////////////////////////
template<typename G>
std::size_t select_source(G const& g)
{
  // Select sources for BFS. Must have >0 outgoing edges.
  boost::random::mt19937 gen;
  boost::random::uniform_int_distribution<size_t> dist(0, g.size()-1);

  for (;;) {
    size_t t = dist(gen);
    if (g[t].size() > 0)
      return t;
  }
}

//////////////////////////////////////////////////////////////////////
/// @brief Create a view over a mesh with a specific vertex property.
//////////////////////////////////////////////////////////////////////
struct mesh_generator
{
  std::size_t m_x, m_y;

  template<typename Property>
  graph_view<graph<UNDIRECTED, MULTIEDGES, Property>>
  generate(void) const
  {
    return stapl::generators::make_mesh<
        graph_view<graph<UNDIRECTED, MULTIEDGES, Property>>
      >(m_x, m_y);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Create a view over an Erdos-Renyi graph with a specific vertex
///         property.
//////////////////////////////////////////////////////////////////////
struct er_generator
{
  std::size_t m_x, m_y;
  double m_p;

  template<typename Property>
  graph_view<graph<UNDIRECTED, MULTIEDGES, Property>>
  generate(void) const
  {
    return stapl::generators::make_erdos_renyi<
        graph_view<graph<UNDIRECTED, MULTIEDGES, Property>>
      >(m_x, m_y, m_p);
  }
};


template<typename Generator>
void test_bfs(Generator&& generator, std::size_t k, double tau)
{
  auto approx_mesh =
    generator.template generate<properties::approximate_bfs_property>();
  auto mesh = generator.template generate<properties::bfs_property>();

  const std::size_t src = select_source(mesh);

  approximate_breadth_first_search(approx_mesh, src, k, tau);
  auto approx_levels = levels(approx_mesh);

  auto exec_policy = stapl::sgl::make_execution_policy("kla", mesh, k);
  breadth_first_search(exec_policy, mesh, src);
  auto exact_levels = levels(mesh);

  const bool correct_levels = stapl::map_reduce(
    validate_level(), stapl::logical_and<bool>(), approx_levels, exact_levels
  );

  STAPL_TEST_REPORT(correct_levels, "Distances of approximate BFS");

  const double max_err =  stapl::map_reduce(
    error_wf(), stapl::max<double>(), approx_levels, exact_levels
  );

  STAPL_TEST_REPORT(max_err <= k, "Error is bounded");
}


stapl::exit_code stapl_main(int argc,char** argv)
{
  if (argc < 5) {
    std::cout<< "usage: exe x-dim y-dim k generator_prob tau\n";
    return EXIT_FAILURE;
  }

  const std::size_t nx = atol(argv[1]);
  const std::size_t ny = atol(argv[2]);
  const std::size_t k = atol(argv[3]);
  const double prob = atof(argv[4]);
  const double tau = atof(argv[5]);

  test_bfs(mesh_generator{nx, ny}, k, tau);
  test_bfs(er_generator{nx, ny, prob}, k, tau);

  return EXIT_SUCCESS;
}

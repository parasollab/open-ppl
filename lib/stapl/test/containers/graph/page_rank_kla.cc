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
#include <stapl/containers/graph/algorithms/page_rank.hpp>
#include <stapl/containers/graph/algorithms/page_rank_kla.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/algorithms/aggregator.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../../test_report.hpp"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Work function to copy a vertex's rank to an array of ranks
//////////////////////////////////////////////////////////////////////
struct copy_rank
{
  using result_type = void;
  template<typename V, typename E>
  result_type operator() (V v, E e)
  {
    e = v.property().rank();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function to determine if two ranks are epsilon close
//////////////////////////////////////////////////////////////////////
struct epsilon_close
{
  using result_type = bool;
  template<typename KLARank, typename LevelSyncRank>
  result_type operator()(KLARank x, LevelSyncRank y)
  {
    return std::fabs(x - y) < 0.001;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to run level synchronous PageRank
//////////////////////////////////////////////////////////////////////
struct run_page_rank
{
  std::size_t m_iters;

  run_page_rank(std::size_t iters)
   : m_iters(iters)
  { }

  template<typename View>
  void operator()(View v) const
  {
    page_rank(v, m_iters);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to run KLA PageRank
//////////////////////////////////////////////////////////////////////
struct run_kla_page_rank
{
  std::size_t m_iters;
  std::size_t m_k;

  run_kla_page_rank(std::size_t iters, std::size_t k)
   : m_iters(iters), m_k(k)
  { }

  template<typename View>
  void operator()(View v) const
  {
    page_rank_kla(v, m_iters, m_k);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Run PageRank on a mesh and return an array of ranks
//////////////////////////////////////////////////////////////////////
template<typename Property, typename PageRankFunc>
array_view<array<double>>
compute_ranks(std::size_t n, PageRankFunc&& f)
{
  using graph_type = graph<stapl::UNDIRECTED, stapl::MULTIEDGES, Property>;
  using view_type = graph_view<graph_type>;

  auto v = stapl::generators::make_mesh<view_type>(n, n);

  auto* results = new array<double>(v.num_vertices());
  array_view<array<double> > results_vw(results);

  std::forward<PageRankFunc>(f)(v);

  map_func(copy_rank(), v, results_vw);

  return results_vw;
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cerr << "usage: exe n k iters" << std::endl;
    exit(1);
  }

  const std::size_t n = atol(argv[1]);
  const std::size_t k = atol(argv[2]);
  const std::size_t iters = atol(argv[3]);

  auto ranks = compute_ranks<stapl::properties::page_rank_property<double>>(
    n, run_page_rank(iters));

  auto ranks_kla = compute_ranks<stapl::properties::page_rank_kla_property>(
    n, run_kla_page_rank(iters, k));

  const double sum = stapl::accumulate(ranks_kla, 0.);

  STAPL_TEST_REPORT(std::fabs(sum - n*n) < 0.000001 , "Ranks sum up to |V|");

  const bool same_ranks =
    map_reduce(epsilon_close(), stapl::logical_and<bool>(), ranks_kla, ranks);

  STAPL_TEST_REPORT(same_ranks, "Same ranks as level-sync");

  return EXIT_SUCCESS;
}

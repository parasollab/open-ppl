/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_GAP_PR_VERIFY_HPP
#define STAPL_BENCHMARKS_GAP_PR_VERIFY_HPP

#include <stapl/algorithms/algorithm.hpp>

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Work-function to return the rank of a vertex
//////////////////////////////////////////////////////////////////////
struct rank_of
{
  using result_type = double;

  template<typename Vertex>
  result_type operator()(Vertex v) const
  {
    return v.property().rank();
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Predicate to determine if a vertex has no outgoing edges.
//////////////////////////////////////////////////////////////////////
struct is_isolated_vertex
{
  using result_type = bool;

  template<typename Vertex>
  result_type operator()(Vertex v) const
  {
    return v.size() == 0;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief In any PageRank run, the sum of the ranks should add up
///        to approximately n.
//////////////////////////////////////////////////////////////////////
template<typename View>
bool verify_page_rank(View vw, double damping)
{
  double sum = stapl::map_reduce(rank_of{}, stapl::plus<double>{}, vw);

  // All isolated vertices will have the rank 1-damping, so we need to add
  // that to the total sum to get up to n.
  const std::size_t isolated_vertices =
    stapl::count_if(vw, is_isolated_vertex{});
  sum += isolated_vertices*(damping);

  const std::size_t n = vw.size();

  return std::fabs(sum - n) < 0.0001;
}

#endif

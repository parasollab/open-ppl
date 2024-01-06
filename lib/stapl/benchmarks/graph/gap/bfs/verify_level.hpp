/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_GAP_BFS_VERIFY_LEVEL_HPP
#define STAPL_BENCHMARKS_GAP_BFS_VERIFY_LEVEL_HPP

#include <stapl/containers/graph/algorithms/paradigms/kla_paradigm.hpp>


//////////////////////////////////////////////////////////////////////
/// @brief Neighbor operator for the verify neighbors graph algorithm
///        for BFS. For every edge in the graph, the two endpoints must
///        satsify the following conditions:
///          1. Their distances differ by at most 1.
///          2. Either both vertices were visited or neither were visited
//////////////////////////////////////////////////////////////////////
class verify_bfl_neighbor_neighbor_op
{
  std::size_t m_source_level;

public:
  verify_bfl_neighbor_neighbor_op(std::size_t level = 0)
    : m_source_level(level)
  { }

  using result_type = bool;

  template<typename V>
  bool operator()(V v) const
  {
    auto target_level = v.property();

    bool levels_differ_by_more_than_one = target_level > m_source_level+1;

    // Either both vertices are discovered or both are undiscovered
    bool same_status =
      (m_source_level == 0 && target_level == 0) ||
      (m_source_level != 0 && target_level != 0);

    if (levels_differ_by_more_than_one || !same_status)
      stapl::abort("Neighbors level relationship invalid.");

    return false;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_source_level);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex operator for the verify neighbors graph algorithms
///        for BFS. Each vertex will visit its neighbors, sending its
///        own level to check if its neighbors' levels make sense.
//////////////////////////////////////////////////////////////////////
struct verify_bfl_neighbor_vertex_op
{
  using result_type = bool;

  template<typename V, typename Vis>
  bool operator()(V v, Vis vis) const
  {
    auto level = v.property();
    vis.visit_all_edges(v, verify_bfl_neighbor_neighbor_op{level});
    return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Verify the results of a BFS tree. Checks to make sure that
///        the discovered levels follow basic sanity checks.
///        Note that if the BFS tree is determined to be invalid, this
///        method will call stapl::abort.
//////////////////////////////////////////////////////////////////////
template<typename G>
bool verify_level(G& g)
{

  // Verify the relationship between a vertex and its neighbors
  stapl::kla_paradigm(
    verify_bfl_neighbor_vertex_op{}, verify_bfl_neighbor_neighbor_op{}, g, 0
  );

  // If control reaches this point without aborting, then verification passed
  return true;
}

#endif

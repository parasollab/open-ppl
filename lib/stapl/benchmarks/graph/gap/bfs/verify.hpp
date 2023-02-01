/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_GAP_BFS_VERIFY_HPP
#define STAPL_BENCHMARKS_GAP_BFS_VERIFY_HPP

#include <stapl/containers/graph/algorithms/paradigms/kla_paradigm.hpp>

//////////////////////////////////////////////////////////////////////
/// @brief Neighbor operator for the verify parent graph algorithm
///        for BFS. For every edge in BFS tree, the parent should
///        satsify the following conditions:
///          1. The parent's distance is one less than the child's
///          2. Both the parent and child should be discovered
//////////////////////////////////////////////////////////////////////
class verify_parent_neighbor_op
{
  std::size_t m_level;

public:
  verify_parent_neighbor_op(std::size_t level = 0)
    : m_level(level)
  { }

  using result_type = bool;

  template<typename V>
  bool operator()(V v) const
  {
    bool correct_level =
      v.property().level() == m_level - 1;

    bool both_discovered =
      v.property().level() > 0 && m_level > 0;

    // If both vertices were discovered, but their distance is off
    // by more than 1, then the traversal is incorrect
    if (both_discovered && !correct_level)
      stapl::abort("Parent-child level relationship invalid.");

    return false;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_level);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Vertex operator for the verify parent graph algorithm
///        for BFS. Each vertex will visit its parent in the BFS tree,
///        sending its own level to check if its parent's level makes sense.
//////////////////////////////////////////////////////////////////////
struct verify_parent_vertex_op
{
  using result_type = bool;

  template<typename V, typename Vis>
  bool operator()(V v, Vis vis) const
  {
    auto level = v.property().level();
    auto parent = v.property().parent();

    // Visit my parent in the BFS tree if it's not myself
    if (parent != v.descriptor())
      vis.visit(parent, verify_parent_neighbor_op{level});

    return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Neighbor operator for the verify neighbors graph algorithm
///        for BFS. For every edge in the graph, the two endpoints must
///        satsify the following conditions:
///          1. Their distances differ by at most 1.
///          2. Either both vertices were visited or neither were visited
//////////////////////////////////////////////////////////////////////
class verify_neighbor_neighbor_op
{
  std::size_t m_source_level;

public:
  verify_neighbor_neighbor_op(std::size_t level = 0)
    : m_source_level(level)
  { }

  using result_type = bool;

  template<typename V>
  bool operator()(V v) const
  {
    auto target_level = v.property().level();

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
struct verify_neighbor_vertex_op
{
  using result_type = bool;

  template<typename V, typename Vis>
  bool operator()(V v, Vis vis) const
  {
    auto level = v.property().level();
    vis.visit_all_edges(v, verify_neighbor_neighbor_op{level});
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
bool verify_bfs(G& g)
{
  // Verify the relationship between a vertex and its parent in the BFS tree
  stapl::kla_paradigm(
    verify_parent_vertex_op{}, verify_parent_neighbor_op{}, g, 0
  );

  // Verify the relationship between a vertex and its neighbors
  stapl::kla_paradigm(
    verify_neighbor_vertex_op{}, verify_neighbor_neighbor_op{}, g, 0
  );

  // If control reaches this point without aborting, then verification passed
  return true;
}

#endif

/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_GAP_SSSP_VERIFY_HPP
#define STAPL_BENCHMARKS_GAP_SSSP_VERIFY_HPP

#include <stapl/containers/graph/algorithms/paradigms/kla_paradigm.hpp>

//////////////////////////////////////////////////////////////////////
/// @brief Neighbor operator for the verify parent graph algorithm
///        for SSSP. For every edge in SSSP tree, the parent should
///        satsify the following conditions:
///          1. The parent's distance is one less than the child's
///          2. Both the parent and child should be discovered
//////////////////////////////////////////////////////////////////////
class verify_parent_neighbor_op
{
  double m_distance;

public:
  verify_parent_neighbor_op(double distance = 0)
    : m_distance(distance)
  { }

  using result_type = bool;

  template<typename V>
  bool operator()(V v) const
  {
    bool greater_distance =
      v.property().distance() > m_distance;

    // If the parent has a greater distance
    // then the traversal is incorrect
    if (greater_distance)
      stapl::abort("Parent-child distance relationship invalid.");

    return false;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_distance);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Vertex operator for the verify parent graph algorithm
///        for SSSP. Each vertex will visit its parent in the SSSP tree,
///        sending its own level to check if its parent's level makes sense.
//////////////////////////////////////////////////////////////////////
struct verify_parent_vertex_op
{
  using result_type = bool;

  template<typename V, typename Vis>
  bool operator()(V v, Vis vis) const
  {
    auto parent = v.property().parent();

    // Visit my parent in the SSSP tree if it's not myself
    if (parent != v.descriptor())
      vis.visit(parent, verify_parent_neighbor_op{v.property().distance()});

    return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Neighbor operator for the verify neighbors graph algorithm
///        for SSSP. For every edge in the graph, the two endpoints must
///        satsify the following conditions:
///          1. Their distances differ by at most 1.
///          2. Either both vertices were visited or neither were visited
//////////////////////////////////////////////////////////////////////
class verify_neighbor_neighbor_op
{
  double m_source_distance;

public:
  verify_neighbor_neighbor_op(double distance = 0.)
    : m_source_distance(distance)
  { }

  using result_type = bool;

  template<typename V>
  bool operator()(V v) const
  {
    auto target_distance = v.property().distance();

    // Either both vertices are discovered or both are undiscovered
    bool same_status =
        (m_source_distance == std::numeric_limits<double>::max()
           && target_distance == std::numeric_limits<double>::max())
      || (m_source_distance != std::numeric_limits<double>::max()
           && target_distance != std::numeric_limits<double>::max());

    if (!same_status)
      stapl::abort("Neighbors level relationship invalid.");

    return false;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_source_distance);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex operator for the verify neighbors graph algorithms
///        for SSSP. Each vertex will visit its neighbors, sending its
///        own level to check if its neighbors' levels make sense.
//////////////////////////////////////////////////////////////////////
struct verify_neighbor_vertex_op
{
  using result_type = bool;

  template<typename V, typename Vis>
  bool operator()(V v, Vis vis) const
  {
    vis.visit_all_edges(
      v, verify_neighbor_neighbor_op{v.property().distance()}
    );
    return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Verify the results of a SSSP tree. Checks to make sure that
///        the discovered levels follow basic sanity checks.
///        Note that if the SSSP tree is determined to be invalid, this
///        method will call stapl::abort.
//////////////////////////////////////////////////////////////////////
template<typename G>
bool verify_sssp(G& g)
{
  // Verify the relationship between a vertex and its parent in the SSSP tree
  stapl::kla_paradigm(
    verify_parent_vertex_op{}, verify_parent_neighbor_op{}, g
  );

  // Verify the relationship between a vertex and its neighbors
  stapl::kla_paradigm(
    verify_neighbor_vertex_op{}, verify_neighbor_neighbor_op{}, g
  );

  // If control reaches this point without aborting, it passed
  return true;
}

#endif

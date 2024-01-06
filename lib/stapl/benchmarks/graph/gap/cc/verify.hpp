/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_GAP_CC_VERIFY_HPP
#define STAPL_BENCHMARKS_GAP_CC_VERIFY_HPP

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/array.hpp>

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief If this vertex receives a label from its neighbor, make sure
///        it's the same label
//////////////////////////////////////////////////////////////////////
class verify_neighor_op
{
  std::size_t m_neighbor_cc;

public:
  verify_neighor_op(std::size_t neighbor_cc = 0)
    : m_neighbor_cc(neighbor_cc)
  { }

  using result_type = bool;

  template<typename Vertex>
  result_type operator()(Vertex v) const
  {
    if (v.property().cc() != m_neighbor_cc)
      stapl::abort("Endpoints of edge have a different label.");

    return false;
  }

  void define_type(typer& t)
  {
    t.member(m_neighbor_cc);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Send connected component label to all neighbors
//////////////////////////////////////////////////////////////////////
struct verify_vertex_op
{
  using result_type = bool;

  template<typename Vertex, typename Visitor>
  result_type operator()(Vertex v, Visitor vis) const
  {
    vis.visit_all_edges(v, verify_neighor_op{v.property().cc()});
    return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Do a quick sanity check that the labels computed by the
///        algorithm make sense. This checks the necessary condition
///        that for all edges (u,v) both u and v share the same label.
///        This is not a sufficient condition, but it will do in a pinch.
/// @param vw The graph view
//////////////////////////////////////////////////////////////////////
template<typename View>
bool verify_cc(View vw)
{
  kla_paradigm(verify_vertex_op{}, verify_neighor_op{}, vw, 0);

  // If control reaches this point without aborting, then it passed
  return true;
}

#endif

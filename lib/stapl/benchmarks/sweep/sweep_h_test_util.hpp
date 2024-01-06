/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_SWEEP_H_TEST_UTIL_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_SWEEP_H_TEST_UTIL_HPP

//////////////////////////////////////////////////////////////////////
/// @brief Functor to compute the property of the new supervertex based
/// on properties of the child vertices.
//////////////////////////////////////////////////////////////////////
struct vpselect_wf
{
  template <class VertexP>
  VertexP operator()(VertexP const& p1, VertexP const&  p2)
  {
    return p1;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Functor to compute the property of the new superedge based
/// on properties of the child edges.
//////////////////////////////////////////////////////////////////////
struct epselect_wf
{
  template <class EdgeP>
  EdgeP operator()(EdgeP const& p1, EdgeP const& p2)
  {
    return p1;
  }
};

#endif

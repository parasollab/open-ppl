/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_CSR_UTILS_HPP
#define STAPL_CONTAINERS_CSR_UTILS_HPP

#include <stapl/containers/graph/csr_graph.hpp>

namespace stapl {

//////////////////////////////////////////////////////////////////////
/// @brief Metafunction to determine whether a graph is a CSR graph.
//////////////////////////////////////////////////////////////////////
template<typename Graph>
struct is_csr
  : std::false_type
{ };

//////////////////////////////////////////////////////////////////////
/// @brief Specialization for CSR graphs
//////////////////////////////////////////////////////////////////////
template<graph_attributes D, graph_attributes M,
         typename VP, typename EP,
         typename PS, typename Map,
         typename Traits>
struct is_csr<csr_graph<D, M, VP, EP, PS, Map, Traits>>
  : std::true_type
{ };


namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to commit/uncommit a graph if it is CSR
//////////////////////////////////////////////////////////////////////
template<typename Graph, bool = is_csr<Graph>::value>
struct try_commit_impl
{
  static void commit(Graph& g)
  {
    g.commit();
  }

  static void uncommit(Graph& g)
  {
    g.uncommit();
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Helper function to do nothing if a graph is not CSR
//////////////////////////////////////////////////////////////////////
template<typename Graph>
struct try_commit_impl<Graph, false>
{
  static void commit(Graph& g)
  { }

  static void uncommit(Graph& g)
  { }
};

} // detail namespace

//////////////////////////////////////////////////////////////////////
/// @brief Commit a graph if it is a CSR representation. If not, do nothing.
/// @param g The graph to commit
//////////////////////////////////////////////////////////////////////
template<typename Graph>
void try_commit(Graph& g)
{
  detail::try_commit_impl<Graph>::commit(g);
}

//////////////////////////////////////////////////////////////////////
/// @brief Uncommit a graph if it is a CSR representation. If not, do nothing.
/// @param g The graph to uncommit
//////////////////////////////////////////////////////////////////////
template<typename Graph>
void try_uncommit(Graph& g)
{
  detail::try_commit_impl<Graph>::uncommit(g);
}

} // stapl namespace

#endif

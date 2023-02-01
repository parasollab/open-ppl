/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_TOPOLOGICAL_SORT_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_TOPOLOGICAL_SORT_HPP

#include <stapl/containers/graph/algorithms/paradigms/graph_paradigm.hpp>


namespace stapl {

namespace topo_algo_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex-initializer functor for @ref topological_sort().
///
/// All vertices have their ranks and preds initialized to zero (0).
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct topological_sort_init_wf
{
  typedef void result_type;
  template<typename Vertex>
  void operator()(Vertex v) const
  {
    v.property().preds(0);
    v.property().rank(0);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Visitor to increment the in-degree of a vertex.
/// Result is stored in the vertex's predecessor-count.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct inc_preds_wf
{
  typedef bool result_type;
  template<typename Vertex>
  bool operator()(Vertex&& target) const
  {
    target.property().preds(target.property().preds() + 1);
    return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to compute the number of incoming edges for
/// each vertex.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct increment_in_degree
{
  typedef bool result_type;

  template<class Vertex, class GraphVisitor>
  result_type operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    graph_visitor.visit_all_edges(std::forward<Vertex>(v), inc_preds_wf());
    return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor for @ref topological_sort().
///
/// Decrements the in-degree (predecessors) by one and updates the
/// target vertex with topological-ordering rank, if the target's
/// current rank is less than the incoming rank.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct update_func
{
  int m_new_rank;

  typedef bool result_type;

  update_func(int nl = 0)
    : m_new_rank(nl)
  { }

  template<typename Vertex>
  bool operator()(Vertex&& target) const
  {
    if (target.property().preds() > 0)
      target.property().preds(target.property().preds() - 1);
    //the rank is updated only if the incoming rank is larger than
    //the existing rank, because we're using async calls, which means
    //that updates from sources with lower levels might reach the vertex
    //after updates from higher level vertices, and the highest rank
    //is the correct topological ordering rank.
    if (m_new_rank > target.property().rank())
      target.property().rank(m_new_rank);

    if (target.property().preds() == 0)
      return true;
    else
      return false;
  }

  void define_type(typer& t)
  { t.member(m_new_rank); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to compute and propagate the topological ordering
/// rank of a vertex.
///
/// A vertex is visited if it is a source (no incoming edges). Source
/// vertices propagate their topological rank +1 to their neighbors,
/// while the neighbor's incoming degree is reduced by one.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct topological_sort_map_wf
{
  typedef bool result_type;

  template<class Vertex, class GraphVisitor>
  result_type operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    if (v.property().preds() == 0) {
      v.property().preds(-1);
      graph_visitor.visit_all_edges(std::forward<Vertex>(v),
                                    update_func(v.property().rank() + 1));
      return true;
    }
    return false;
  }
};

} // namespace topo_algo_detail.


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Topological Sort algorithm.
///
/// The input graph must be a Directed Acyclic Graph (DAG).
/// Performs a topological ordering on the input @ref graph_view, storing
/// the topological ordering in the vertex's rank property.
/// This algorithm iteratively finds source-vertices and assigns the next
/// level in the topological ordering to neighboring vertices, while
/// decrementing their in-degrees. When the in-degree reaches zero (0),
/// the vertex becomes a source vertex.
/// @param gvw The @ref graph_view over the input graph.
/// @param k The maximum amount of asynchrony allowed in each phase.
/// 0 <= k <= inf.
/// k == 0 implies level-synchronous topological sort.
/// k >= D implies fully asynchronous topological sort (D is diameter of graph).
///
/// @todo Add another function that will produce an @ref array_view whose
/// elements represent a valid topological sort using the topological
/// levels computed.  This is needed by PDT and other applications that
/// need a topological order that they can use to enforce ordering.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <typename GraphView>
void topological_sort(GraphView& gvw, size_t k=0)
{
  using namespace topo_algo_detail;

  //the rank and the in-degree of each vertex are initialized to zero
  map_func(topological_sort_init_wf(), gvw);

  //at each vertex, increment the number of predecessors of each neighbor
  //vertex by one
  graph_paradigm(increment_in_degree(), inc_preds_wf(), gvw, 0);

  //while there are source vertices:
  //  At each source vertex decrement neighbors' in-degree (to eventually create
  //  new source vertices) and send the rank+1 of a source node to its
  //  neighbors, which can be the new rank of the neighbor.
  //  At the neighbor side, if the passed rank is larger than the old rank
  //  value the new value will be stored

  graph_paradigm(topological_sort_map_wf(), update_func(), gvw, k);
}

}//namespace stapl
#endif

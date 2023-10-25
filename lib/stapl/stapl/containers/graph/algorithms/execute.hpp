/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_EXECUTE_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_EXECUTE_HPP

#include <stapl/containers/graph/algorithms/execution_policy.hpp>

#include <stapl/containers/graph/algorithms/paradigms/graph_paradigm.hpp>
#include <stapl/containers/graph/algorithms/paradigms/h_paradigm.hpp>
#include <stapl/containers/graph/algorithms/paradigms/h2_paradigm.hpp>
#include <stapl/containers/graph/algorithms/paradigms/h_hubs_paradigm.hpp>

#include <boost/variant/variant.hpp>

namespace stapl {

namespace sgl {

//////////////////////////////////////////////////////////////////////
/// @brief A dummy vertex-property reducer for algorithms that don't use
/// the hubs policy.
//////////////////////////////////////////////////////////////////////
struct noop_vp_reducer
{
  template<typename VP1, typename VP2>
  void operator()(VP1&, VP2&) const
  {
    stapl::abort("Please provide a valid vertex-property reducer to use hubs");
  }
};

namespace detail {

//////////////////////////////////////////////////////////////////////
/// @brief Helper to invoke the correct execution strategy based on the
/// provided policy.
///
/// @tparam VertexOp Type of the user provided vertex-operator.
/// @tparam NeighborOp The type of the user provided neighbor-operator
/// expressing the computation to be performed over neighboring vertices.
/// @tparam Reducer Type of the  vertex property reducer for the algorithm.
/// @tparam FinishPredicate Type of the predicate indicating when to terminate
/// a traversal.
/// @tparam PostExecute Type of functor that will be executed after each
/// iteration.
/// @tparam View Type of the input @ref graph_view.
//////////////////////////////////////////////////////////////////////
template<typename VertexOp, typename NeighborOp, typename Reducer,
         typename FinishPredicate, typename PostExecute, typename View>
struct execute_visitor
  : boost::static_visitor<std::size_t>
{
  VertexOp& m_vertex_op;
  NeighborOp& m_neighbor_op;
  View& m_view;
  Reducer m_reducer;
  FinishPredicate m_pred;
  PostExecute m_post_execute;

  execute_visitor(VertexOp& vertex_op, NeighborOp& neighbor_op,
                  Reducer&& reducer, FinishPredicate&& finish_pred,
                  PostExecute&& post_execute, View& view)
    : m_vertex_op(vertex_op), m_neighbor_op(neighbor_op), m_view(view),
      m_reducer(reducer), m_pred(finish_pred), m_post_execute(post_execute)
  { }

  std::size_t operator()(sgl::kla_policy& policy) const
  {
    kla_params<View, FinishPredicate> params;
    params.predicate = m_pred;
    return kla_paradigm(m_vertex_op, m_neighbor_op, m_view, policy.k(), params);
  }

  std::size_t operator()(sgl::level_sync_policy& policy) const
  {
    kla_params<View, FinishPredicate> params;
    params.post_execute = m_post_execute;
    params.predicate = m_pred;

    return kla_paradigm(m_vertex_op, m_neighbor_op,
                               m_view, 0, params);
  }

  std::size_t operator()(sgl::async_policy& policy) const
  {
    return kla_paradigm(m_vertex_op, m_neighbor_op, m_view,
                        std::numeric_limits<std::size_t>::max()-1);
  }

  std::size_t operator()(sgl::hierarchical_policy<View>& policy) const
  {
    return h_paradigm(m_vertex_op, m_neighbor_op, m_post_execute,
                      policy.h_view(), m_view);
  }

  std::size_t operator()(sgl::hierarchical_hubs_policy<View>& policy) const
  {
    return h_hubs_paradigm(m_vertex_op, m_neighbor_op, m_reducer,
                           m_post_execute, policy.h_view(), policy.hubs_view(),
                           m_view);
  }

  template<typename T>
  std::size_t operator()(T&&) const
  {
    stapl::abort("Execution policy not supported for sgl::execute");
    return 0;
  }
};

} // namespace detail

//////////////////////////////////////////////////////////////////////
/// @brief The SGL execute method. This executes the given operators
/// on the graph based on the provided policy.
///
/// The user provides a vertex-operator to express the computation to be
/// performed on each vertex, and a neighbor-operator that will be applied
/// to each neighbor that is visited.
/// The vertex-operator is passed in a vertex and a visit object. To visit a
/// neighboring vertex, the vertex-operator must call
/// visit(neighbor, neighbor-operator()).
/// The vertex-operator must return true if the vertex was active
/// (i.e. its value was updated), or false otherwise.
/// The neighbor-operator is passed in the target vertex. Neighbor-operators
/// may carry state, but must be immutable. They should return true if the
/// visit was successful (i.e. the target vertex will be activated after this
/// visit), or false otherwise.
/// Users may also provide additional functions to be executed after each SS.
/// @param policy A policy for execution.
/// @param view The @ref graph_view over the input graph.
/// @param vertex_op Functor that implements the operation to be performed
/// on each vertex
/// @param neighbor_op The type of the user provided neighbor-operator
/// expressing computation to be performed over neighboring vertices.
/// @param reducer A vertex property reducer for the algorithm. It should accept
/// two vertex properties and reduce them to update the first one. Used to
/// update the hub vertices.
/// @param finish_pred A predicate indicating when to terminate that
/// receives a vertex
/// @param post_execute Optional functor that will be executed on the
/// @ref graph_view at the end of each SS. This will be invoked with the
/// input @ref graph_view and the current SS ID (the ID of the SS that
/// just finished).
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Policy, typename View, typename VertexOp, typename NeighborOp,
         typename Reducer = noop_vp_reducer,
         typename FinishPredicate = kla_detail::false_predicate,
         typename PostExecute = kla_detail::empty_prepost_execute>
std::size_t execute(Policy&& policy, View& view, VertexOp&& vertex_op,
                    NeighborOp&& neighbor_op, Reducer reducer = Reducer(),
                    FinishPredicate finish_pred = FinishPredicate(),
                    PostExecute post_execute = PostExecute())
{
  return boost::apply_visitor(
    detail::execute_visitor<VertexOp, NeighborOp, Reducer, FinishPredicate,
      PostExecute, View>{ vertex_op, neighbor_op, std::move(reducer),
                          std::move(finish_pred), std::move(post_execute),
                          view },
    policy
  );
}

} // namespace sgl

} // stapl namespace

#endif

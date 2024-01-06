/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_GRAPH_PARADIGM_PG_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_PARADIGMS_GRAPH_PARADIGM_PG_HPP

#include "kla_paradigm.hpp"
#include <stapl/containers/graph/views/graph_view.hpp>

namespace stapl {

namespace kla_pg {

template<typename WF, typename PostExecute, typename GView,
         typename Predicate = kla_pg_detail::false_predicate>
struct graph_paradigm_params {

  WF uwf;

  PostExecute post_execute;

  GView g;
  /// @brief The Parallel Graph Algorithm Paradigm parameters.
  /// The maximum amount of asynchrony allowed in each phase.
  /// 0 <= k <= inf.
  /// k == 0 implies level-synchronous paradigm.
  /// k >= D implies fully asynchronous paradigm (D is the number of iterations
  /// performed by the level-synchronous variant of the algorithm).
  int k;


  Predicate predicate;

  graph_paradigm_params():
  post_execute(kla_pg_detail::empty_prepost_execute()), k(0) {}
};

//////////////////////////////////////////////////////////////////////
/// @brief The Parallel Graph Algorithm Paradigm.
///
/// Allows expression of parallel fine-grained graph algorithms, with
/// optimizations for selecting between different graph algorithmic paradigms.
///
/// The user provides a work function to express the computation to be performed
/// on each vertex, and a visitor that will be applied to each neighbor that is
/// visited.
/// The work function is passed in a vertex and a visit object. To visit a
/// neighboring vertex, the work function must call visit(neighbor, visitor()).
/// The work function must return true if the vertex was active (i.e. its value
/// was updated), or false otherwise.
/// The visitor is passed in the target vertex. Visitors may carry state, but
/// must be immutable. They should return true if the visit was successful (i.e.
/// the target vertex will be activated after this visit), or false otherwise.
/// Users may also provide additional functions to be executed after each
/// KLA-Superstep (KLA-SS).
/// @tparam WF The type of the user provided work function expressing
/// computation to be performed over each vertex.
/// @tparam UF The type of the user provided visitor expressing computation to
/// be performed over neighboring vertices.
/// @param uwf Functor that implements the operation to be performed
/// on each vertex
/// @param post_execute Optional functor that will be executed on the
/// @ref graph_view at the end of each KLA-SS. This will be invoked with
/// the input @ref graph_view and the current KLA-SS ID (the ID of the
/// KLA-SS that just finished).
/// @param g The @ref graph_view over the input graph.
/// @param k The maximum amount of asynchrony allowed in each phase.
/// 0 <= k <= inf.
/// k == 0 implies level-synchronous paradigm.
/// k >= D implies fully asynchronous paradigm (D is the number of iterations
/// performed by the level-synchronous variant of the algorithm).
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
/// @todo Select between lsync, KLA, async, and adaptive paradigms based
/// on k. -1 (default) can imply adaptive, 0 lsync, >0 KLA,
/// and MAX_INT async.
//////////////////////////////////////////////////////////////////////
template<typename WF, typename UF, typename PostExecute, typename GView>
size_t graph_paradigm_pg(WF const& uwf, UF const&, PostExecute post_execute,
			 GView& g, int k)
{
  {
    kla_params<GView,kla_pg_detail::false_predicate> params;
    params.post_execute = post_execute;
    params.predicate = kla_pg_detail::false_predicate();

    return kla_paradigm_pg(uwf, UF(), g, k, params);
  }
}

//////////////////////////////////////////////////////////////////////
/// @brief The Parallel Graph Algorithm Paradigm.
///
/// Allows expression of parallel fine-grained graph algorithms, with
/// optimizations for selecting between different graph algorithmic paradigms.
///
/// The user provides a work function to express the computation to be performed
/// on each vertex, and a visitor that will be applied to each neighbor that is
/// visited.
/// The work function is passed in a vertex and a visit object. To visit a
/// neighboring vertex, the work function must call visit(neighbor, visitor()).
/// The work function must return true if the vertex was active (i.e. its value
/// was updated), or false otherwise.
/// The visitor is passed in the target vertex. Visitors may carry state, but
/// must be immutable. They should return true if the visit was successful (i.e.
/// the target vertex will be activated after this visit), or false otherwise.
/// Users may also provide additional functions to be executed after each
/// KLA-Superstep (KLA-SS).
/// @tparam WF The type of the user provided work function expressing
/// computation to be performed over each vertex.
/// @tparam UF The type of the user provided visitor expressing computation to
/// be performed over neighboring vertices.
/// @tparam Predicate A predicate used by BFS to check for a target vertex.
/// @param uwf Functor that implements the operation to be performed
/// on each vertex
/// @param post_execute Optional functor that will be executed on the
/// @ref graph_view at the end of each KLA-SS. This will be invoked with
/// the input @ref graph_view and the current KLA-SS ID (the ID of the
/// KLA-SS that just finished).
/// @param g The @ref graph_view over the input graph.
/// @param k The maximum amount of asynchrony allowed in each phase.
/// 0 <= k <= inf.
/// k == 0 implies level-synchronous paradigm.
/// k >= D implies fully asynchronous paradigm (D is the number of iterations
/// performed by the level-synchronous variant of the algorithm).
/// @param predicate The predicate to be passed to the level_sync and kla
/// paradigms
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
/// @todo Select between lsync, KLA, async, and adaptive paradigms based
/// on k. -1 (default) can imply adaptive, 0 lsync, >0 KLA,
/// and MAX_INT async.
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
template<typename WF, typename UF, typename GView, typename PostExecute,
         typename Predicate>
size_t graph_paradigm_pg(UF const& ,
    graph_paradigm_params<WF, PostExecute, GView, Predicate>& params)
{
  {
    kla_params<GView, Predicate> in_params;
    in_params.post_execute = params.post_execute;
    in_params.predicate = params.predicate;

    return kla_paradigm_pg(params.uwf, UF(), params.g, params.k, in_params);
  }

}

//////////////////////////////////////////////////////////////////////
/// @brief The Parallel Graph Algorithm Paradigm.
///
/// Overloaded variant of @ref graph_paradigm with an empty post-execute.
//////////////////////////////////////////////////////////////////////
template<typename WF, typename UF, typename GView>
size_t graph_paradigm_pg(WF const& uwf, UF const&, GView& g, int k=0)
{
  return graph_paradigm_pg(uwf, UF(),
			   kla_pg_detail::empty_prepost_execute(), g, k);
}

} // namespace kla_pg

} // namespace stapl

#endif

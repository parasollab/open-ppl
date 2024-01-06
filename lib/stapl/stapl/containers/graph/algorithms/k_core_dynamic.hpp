/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_K_CORE_DYNAMIC_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_K_CORE_DYNAMIC_HPP

#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/views/lazy_graph_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/algorithms/algorithm.hpp>

namespace stapl {

namespace k_core_algo_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Functor to delete vertices whose out-degree does not
/// satisfy the comparison with k.
/// @tparam Comp Type of the comparator.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename Comp>
class k_core_delete_wf
{
  size_t m_k;
  Comp   m_comp;

public:
  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param k The 'k' parameter in k-core.
  /// @param comp The comparator object.
  //////////////////////////////////////////////////////////////////////
  k_core_delete_wf(size_t k, Comp comp)
    : m_k(k), m_comp(comp)
  { }

  template <typename T, typename LazyGVw>
  result_type operator()(T v, LazyGVw lgv) const
  {
    if (m_comp(v.size(), m_k)) {
      lgv.delete_vertex(v.descriptor());
      return true;
    }
    return false;
  }

  void define_type(typer& t)
  {
    t.member(m_k);
    t.member(m_comp);
  }
};

}; // namespace k_core_algo_detail


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel k-core algorithm.
/// Iteratively deletes all vertices not satisfying the specified comparison
/// of the vertex's out-degree with the provided k parameter.
/// @param graph The @ref graph_view over the input graph.
/// @param k The parameter to control which vertices are deleted.
/// @param comp The comparator to compare the vertex's out-degree with k.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GView, typename Comp>
size_t k_core_dynamic(GView& graph, size_t k, Comp comp)
{
  size_t iter_deleted = 1, num_deleted = 0;
  while (iter_deleted != 0 && graph.size() != 0)
  {
    iter_deleted
      = map_reduce(k_core_algo_detail::k_core_delete_wf<Comp>(k, comp),
                   stapl::plus<size_t>(), GView(graph.container()),
                   make_repeat_view(lazy_graph(graph.container())));
    // required due to delete vertices called through lazy-view in map-reduce.
    rmi_fence();
    num_deleted += iter_deleted;
  }

  return num_deleted;
}


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel k-core algorithm.
/// Iteratively deletes all vertices with out-degree smaller than k.
/// @param graph The @ref graph_view over the input graph.
/// @param k The parameter to control which vertices are deleted.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GView>
size_t k_core_dynamic(GView& graph, size_t k)
{
  return k_core_dynamic(graph, k, stapl::less<size_t>());
}

} // namespace stapl
#endif

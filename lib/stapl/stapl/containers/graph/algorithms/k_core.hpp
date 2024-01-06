/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_K_CORE_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_K_CORE_HPP

#include <stapl/containers/graph/algorithms/execute.hpp>

namespace stapl {

namespace k_core_algo_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor for @ref k_core() initializer.
///
/// Increments the target vertex's property by one to obtain the number
/// of incoming edges to it.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct init_update_func
{
  typedef bool result_type;

  template <class Vertex>
  result_type operator()(Vertex&& target) const
  {
    ++target.property();
    return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex-initializer functor for @ref k_core().
///
/// Initializes vertices' property to its degree.
/// -1 on the vertex property denotes a 'deleted' vertex.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct init_prop_wf
{
  typedef bool result_type;

  template<typename Vertex, typename GraphVisitor>
  result_type operator() (Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    // If the first step, set all properties to zero.
    if (graph_visitor.level() == 1) {
      v.property() = 0;
      return true;
    } else {
      graph_visitor.visit_all_edges(v, init_update_func());
      return false;
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reducer functor for @ref k_core().
///
/// Reduces two k_core properties to update the first one.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct vp_reducer
{
  template<typename VP1, typename VP2>
  void operator()(VP1& p1, VP2& p2) const
  {
    // Only update non-deleted vertices.
    if (p1 != -1)
      p1 += p2;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor for @ref k_core().
///
/// Decrements the target vertex's property by one. Target vertex becomes
/// active when property is zero.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct update_func
{
  typedef bool result_type;

  template <class Vertex>
  bool operator()(Vertex&& target) const
  {
    if (target.property() >= 0) {
      --target.property();
      return true;
    }
    return false;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function for @ref k_core().
///
/// A vertex is visited if it is active, i.e. its property satisfies
/// the provided comparator. Active vertices decrement their neighbors'
/// property (in-degree), and are then marked as deleted.
/// Returns true if vertex was active, false otherwise.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<class Comp>
struct k_core_wf
{
  int m_core_sz;
  Comp   m_comp;

  typedef bool result_type;

  k_core_wf(int core_size, Comp const& comp)
    : m_core_sz(core_size), m_comp(comp)
  { }

  template<typename Vertex, typename GraphVisitor>
  bool operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    if (v.property() >= 0 && m_comp(v.property(), m_core_sz)) {
      // mark v as deleted.
      v.property() = -1;
      graph_visitor.visit_all_edges(std::forward<Vertex>(v), update_func());
      return true;
    }
    return false;
  }

  void define_type(typer& t)
  {
    t.member(m_core_sz);
    t.member(m_comp);
  }
}; // struct k_core_wf

} // namespace k_core_tunable_algo_detail


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel k-core algorithm.
///
/// Iteratively marks all vertices not satisfying the specified comparison
/// of the vertex's out-degree with the provided k parameter, as 'deleted',
/// i.e. the vertex property is set to -1.
/// @param policy A policy for execution
/// @param graph The @ref graph_view over the input graph.
/// @param core_sz The parameter to control which vertices are deleted.
/// @param comp The comparator to compare the vertex's out-degree with k.
/// @return The number of iterations performed by the paradigm. Note this
/// is different from the dynamic version which returns the number of
/// vertices deleted.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Policy, typename GView, typename Comp>
size_t k_core(Policy&& policy, GView& graph, int core_sz, Comp comp)
{
  using namespace k_core_algo_detail;

  k_core_wf<Comp> wf(core_sz, comp);

  // Initialize each vertex's property to reflect its adjacency size.
  sgl::execute(
    std::forward<Policy>(policy), graph, init_prop_wf(), init_update_func(),
    vp_reducer{}
  );

  return sgl::execute(
      std::forward<Policy>(policy), graph, wf, update_func(), vp_reducer{}
  );
}


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel k-core algorithm.
///
/// A variant of @ref k_core() that uses the less<int>() comparator.
/// @param policy A policy for execution.
/// @param graph The @ref graph_view over the input graph.
/// @param core_sz The parameter to control which vertices are deleted.
/// @return The number of iterations performed by the paradigm. Note this
/// is different from the dynamic version which returns the number of
/// vertices deleted.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Policy, typename GView>
size_t k_core(Policy&& policy, GView& graph, int core_sz)
{
  return k_core(policy, graph, core_sz, stapl::less<int>());
}

} // namespace stapl

#endif

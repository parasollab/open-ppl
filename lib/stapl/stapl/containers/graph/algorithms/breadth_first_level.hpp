/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_BREADTH_FIRST_LEVEL_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_BREADTH_FIRST_LEVEL_HPP

#include <stapl/containers/graph/algorithms/execute.hpp>

namespace stapl {

namespace bfl_algo_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex-initializer functor for @ref breadth_first_level().
///
/// Initializes vertices' BFS-level.
/// If the vertex is the source, the level is one (active).
/// All other vertices' levels are set to zero (inactive).
/// @tparam VD Type of the vertex-descriptor.
/// @tparam LevelType Type of the vertex-level.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <class VD>
class bfl_init_wf
{
  typedef VD vd_type;
  vd_type    m_source;

public:
  typedef void result_type;

  bfl_init_wf(vd_type source)
    : m_source(source)
  { }

  template <class Vertex>
  void operator()(Vertex&& v)
  {
    if (v.descriptor() == m_source) {
      v.property() = 1;
    } else {
      v.property() = 0;
    }
  }

  void define_type(typer& t)
  { t.member(m_source); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reducer functor for @ref breadth_first_level().
///
/// Reduces two BFS properties to update the first one.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct vp_reducer
{
  template<typename VP1, typename VP2>
  void operator()(VP1& p1, VP2& p2) const
  {
    if (p1 == 0 || p1 > p2) {
      p1 = p2;
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor for @ref breadth_first_level().
///
/// Updates the target vertex with BFS-level information,
/// if the target vertex has not been visited before, or
/// if the target's level is greater than the incoming level.
/// @tparam VD Type of the vertex-descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class update_func
{
public:
  size_t            m_level;

  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param level The BFS-level of the target vertex.
  //////////////////////////////////////////////////////////////////////
  update_func(size_t level = 0)
    : m_level(level)
  { }

  template <class Vertex>
  result_type operator()(Vertex&& target) const
  {
    if (target.property() == 0 ||
        target.property() > m_level) {
      target.property() = m_level;
      return true;
    }
    //else ignore.
    return false;
  }

  void define_type(typer& t)
  {
    t.member(m_level);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function for @ref breadth_first_level().
/// A vertex is visited if it is active (the level on the vertex matches the
/// current level of the paradigm. Active vertices update their neighbors with
/// new BFS level information.
/// Returns true if vertex was active, false otherwise.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct bfl_map_wf
{
  using concurrency_model = sgl::weak_concurrency;

  template<class Vertex, class GraphVisitor>
  bool operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    if (v.property() == graph_visitor.level()) {  // if v is active.
      graph_visitor.visit_all_edges(v,
        update_func(v.property()+1));
      return true;
    }
    return false;
  }
};
} // namespace bfl_algo_detail


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Breadth-First Search (BFS) that computes level
///        only.
///
/// Performs a breadth-first search on the input @ref graph_view, storing
/// the BFS-level on each reachable vertex.
/// @param policy A policy for execution.
/// @param g The @ref graph_view over the input graph.
/// @param source The descriptor of the source vertex for this traversal.
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Policy, typename View>
size_t breadth_first_level(Policy&& policy, View& g,
                            typename View::vertex_descriptor const& source)
{
  using namespace bfl_algo_detail;

  using vd_type = typename View::vertex_descriptor;
  using neighbor_op = update_func;
  using vertex_op = bfl_map_wf;

  // Initialize the vertices.
  map_func(bfl_init_wf<vd_type>(source), g);

  return sgl::execute(
    std::forward<Policy>(policy), g, vertex_op{}, neighbor_op{}, vp_reducer{}
  );
}

//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Breadth-First Search (BFS) that computes level
///        only.
///
/// Performs a breadth-first search on the input @ref graph_view, storing
/// the BFS-level on each reachable vertex.
/// @param g The @ref graph_view over the input graph.
/// @param source The descriptor of the source vertex for this traversal.
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename View>
size_t breadth_first_level(View& g,
                            typename View::vertex_descriptor const& source)
{
  auto exec_policy = sgl::execution_policy<View>{sgl::level_sync_policy{}};
  return breadth_first_level(exec_policy, g, source);
}

} // namespace stapl

#endif

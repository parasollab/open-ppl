/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_BREADTH_FIRST_SEARCH_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_BREADTH_FIRST_SEARCH_HPP

#include <stapl/containers/graph/algorithms/execute.hpp>

namespace stapl {

namespace bfs_algo_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex-initializer functor for @ref breadth_first_search().
///
/// Initializes vertices' BFS-parent, BFS-level.
/// If the vertex is the source, the level is one (active).
/// All other vertices' levels are set to zero (inactive).
/// All vertices' BFS-parents are set to their own descriptors.
/// @tparam VD Type of the vertex-descriptor.
/// @tparam LevelType Type of the vertex-level.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <class VD, class LevelType>
class bfs_init_wf
{
  typedef VD vd_type;
  vd_type    m_source;

public:
  typedef void result_type;

  bfs_init_wf(vd_type source)
    : m_source(source)
  { }

  template <class Vertex>
  void operator()(Vertex v)
  {
    if (v.descriptor() == m_source) {
      v.property().level(LevelType(1));
    } else {
      v.property().level(LevelType(0));
    }
    v.property().parent(v.descriptor());
  }

  void define_type(typer& t)
  { t.member(m_source); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Reducer functor for @ref breadth_first_search().
///
/// Reduces two BFS properties to update the first one.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct vp_reducer
{
  template<typename VP1, typename VP2>
  void operator()(VP1& p1, VP2& p2) const
  {
    if (p1.level() == 0 || p1.level() > p2.level()) {
      p1.level(p2.level());
      p1.parent(p2.parent());
    }
  }
};


struct false_predicate
{
  template<typename Vertex>
  bool operator()(Vertex&&) const
  { return false;}
};

//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor for @ref breadth_first_search().
///
/// Updates the target vertex with BFS-parent and BFS-level
/// information, if the target vertex has not been visited before, or
/// if the target's level is greater than the incoming level.
/// @tparam VD Type of the vertex-descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VD>
class update_func
{
public:
  typedef VD        parent_type;
  parent_type       m_parent;
  size_t            m_level;

  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param p The BFS-parent of the target vertex.
  /// @param level The BFS-level of the target vertex.
  //////////////////////////////////////////////////////////////////////
  update_func(parent_type p = 0, size_t level = 0)
    : m_parent(p), m_level(level)
  { }

  template <class Vertex>
  result_type operator()(Vertex&& target) const
  {
    if (target.property().level() == 0 ||
        target.property().level() > m_level) {
      target.property().level(m_level);
      target.property().parent(m_parent);
      return true;
    }
    //else ignore.
    return false;
  }

  void define_type(typer& t)
  {
    t.member(m_parent);
    t.member(m_level);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function for @ref breadth_first_search().
/// A vertex is visited if it is active (the level on the vertex matches the
/// current level of the paradigm. Active vertices update their neighbors with
/// new BFS level and parent information.
/// Returns true if vertex was active, false otherwise.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct bfs_map_wf
{
  using concurrency_model = sgl::weak_concurrency;

  template<class Vertex, class GraphVisitor>
  bool operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    using descriptor_type =
      typename std::decay<Vertex>::type::vertex_descriptor;

    if (v.property().level() == graph_visitor.level()) {  // if v is active.
      graph_visitor.visit_all_edges(std::forward<Vertex>(v),
        update_func<descriptor_type>(v.descriptor(), v.property().level()+1));
      return true;
    }
    return false;
  }
};
} // namespace bfs_algo_detail


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Breadth-First Search (BFS)
///
/// Performs a breadth-first search on the input @ref graph_view, storing
/// the BFS-level and BFS-parent on each reachable vertex.
/// @param policy A policy for execution.
/// @param g The @ref graph_view over the input graph.
/// @param source The descriptor of the source vertex for this traversal.
/// @param finish_pred A predicate indicating when to terminate that
///        receives a vertex.
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Policy, typename View, typename Predicate>
size_t breadth_first_search(Policy&& policy, View& g,
                            typename View::vertex_descriptor const& source,
                            Predicate&& finish_pred)
{
  using namespace bfs_algo_detail;

  using vd_type = typename View::vertex_descriptor;
  using neighbor_op = update_func<vd_type>;
  using vertex_op = bfs_map_wf;

  // Initialize the vertices.
  map_func(bfs_init_wf<vd_type, size_t>(source), g);

  return sgl::execute(
    std::forward<Policy>(policy), g, vertex_op{}, neighbor_op{}, vp_reducer{},
    std::forward<Predicate>(finish_pred)
  );
}

//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Breadth-First Search (BFS)
///
/// Performs a breadth-first search on the input @ref graph_view, storing
/// the BFS-level and BFS-parent on each reachable vertex.
/// @param policy A policy for execution.
/// @param g The @ref graph_view over the input graph.
/// @param source The descriptor of the source vertex for this traversal.
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Policy, typename View>
size_t breadth_first_search(Policy&& policy, View& g,
                            typename View::vertex_descriptor const& source)
{
  auto finish_pred = bfs_algo_detail::false_predicate{};
  return breadth_first_search(std::forward<Policy>(policy), g, source,
                              finish_pred);
}

//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Breadth-First Search (BFS)
///
/// Performs a breadth-first search on the input @ref graph_view, storing
/// the BFS-level and BFS-parent on each reachable vertex.
/// @param g The @ref graph_view over the input graph.
/// @param source The descriptor of the source vertex for this traversal.
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename View>
size_t breadth_first_search(View& g,
                            typename View::vertex_descriptor const& source)
{
  auto exec_policy = sgl::execution_policy<View>{sgl::level_sync_policy{}};
  auto finish_pred = bfs_algo_detail::false_predicate{};
  return breadth_first_search(exec_policy, g, source, finish_pred);
}

} // namespace stapl

#endif

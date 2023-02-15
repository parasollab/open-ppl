/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_MAXIMAL_BIPARTITE_MATCHING_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_MAXIMAL_BIPARTITE_MATCHING_HPP

#include <stapl/containers/graph/algorithms/paradigms/graph_paradigm.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>

namespace stapl {

namespace mbm_algo_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Indicates the type of the matching request.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
enum op_type
{ NO_OP, REQUEST_MATCH, INFORM_MATCH, FINAL_MATCH };


//////////////////////////////////////////////////////////////////////
/// @brief Vertex-initializer functor for @ref maximal_bipartite_matching().
///
/// Initializes vertices' match, level, and sets it as unmatched.
/// All vertices' levels are set to one (active in the first iteration).
/// All vertices are set to unmatched.
/// All vertices' matches are set to invalid descriptors.
/// @tparam VD Type of the vertex-descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <class VD>
class mbm_init_wf
{
public:
  typedef void result_type;

  template <class Vertex>
  result_type operator()(Vertex v)
  {
    v.property().level(1);
    v.property().matched(false);
    v.property().match(std::numeric_limits<VD>::max());
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex-finalizer functor for @ref maximal_bipartite_matching().
///
/// If the vertex is not matched, the match is reset to undefined.
/// @tparam VD Type of the vertex-descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <class VD>
class mbm_final_wf
{
public:
  typedef size_t result_type;

  template <class Vertex>
  result_type operator()(Vertex v)
  {
    if (!v.property().matched()) {
      v.property().match(std::numeric_limits<VD>::max());
      return 0;
    }
    return 1;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor for @ref maximal_bipartite_matching().
///
/// Updates the target vertex with parent-match and level
/// information
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
  op_type           m_action;

  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param p The parent-match of the target vertex.
  /// @param level The level of the parent-match vertex.
  //////////////////////////////////////////////////////////////////////
  update_func(parent_type p = 0, size_t level = 0,
              op_type const& action = op_type::NO_OP)
    : m_parent(p), m_level(level), m_action(action)
  { }

  template <class Vertex>
  result_type operator()(Vertex&& target) const
  {
    target.property().level(m_level + 1);

    if (m_action == op_type::REQUEST_MATCH) {
      if (!target.property().matched())
        target.property().match(m_parent);
    } else if (m_action == op_type::INFORM_MATCH &&
               target.property().match()
               == std::numeric_limits<size_t>::max()) {
      target.property().match(m_parent);
      target.property().matched(true);
    } else if (m_action == op_type::FINAL_MATCH) {
      target.property().match(m_parent);
      target.property().matched(true);
      target.property().level(0);
    }
    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_parent);
    t.member(m_action);
    t.member(m_level);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function for @ref maximal_bipartite_matching().
/// A vertex is visited if it is active (the level on the vertex matches the
/// current level of the paradigm. Active vertices update their neighbors
/// according to the maximal matching algorithm.
/// Returns true if vertex was active, false otherwise.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct mbm_map_wf
{
  typedef bool result_type;

  template<class Vertex, class GraphVisitor>
  result_type operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    using descriptor_t = typename std::decay<Vertex>::type::vertex_descriptor;

    if (v.property().level() == graph_visitor.level()) {
      if (v.property().partition() == properties::bipartite_partition::LEFT &&
          !v.property().matched()) {
        graph_visitor.visit_all_edges(v,
          update_func<descriptor_t>(v.descriptor(), graph_visitor.level(),
                                    op_type::REQUEST_MATCH));
      } else if (v.property().partition()
                 == properties::bipartite_partition::RIGHT &&
                 !v.property().matched() &&
                 v.property().match() != std::numeric_limits<size_t>::max()) {
        graph_visitor.visit_all_edges(v,
          update_func<descriptor_t>(v.descriptor(), graph_visitor.level(),
                                    op_type::NO_OP));
        graph_visitor.visit(v.property().match(),
          update_func<descriptor_t>(v.descriptor(), graph_visitor.level(),
                                    op_type::INFORM_MATCH));
      } else if (v.property().partition()
                 == properties::bipartite_partition::LEFT &&
                 v.property().matched()) {
        graph_visitor.visit(v.property().match(),
          update_func<descriptor_t>(v.descriptor(), graph_visitor.level(),
                                    op_type::FINAL_MATCH));
      }
      v.property().level(0);
      return true;
    }
    return false;
  }
};

} // namespace mbm_algo_detail


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Maximal Bipartite Matching (MBM)
///
/// Performs a maximal bipartite matching on the input @ref graph_view,
/// storing the matchings on each vertex.
/// @param g The @ref graph_view over the input graph. The input must be
/// a bipartite graph, with the vertices divided in one of
/// properties::bipartite_partition::LEFT or
/// properties::bipartite_partition::RIGHT,
/// as specified by the vertex property. Edges should only occur between
/// vertices of different partitions, i.e., inter-partition.
/// @return The size of the maximal matching.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<class GView>
size_t maximal_bipartite_matching(GView& g, size_t k = 0)
{
  using namespace mbm_algo_detail;
  typedef typename GView::vertex_descriptor vd_type;
  typedef update_func<vd_type> update_func_t;

  // Initialize the vertices.
  map_func(mbm_init_wf<vd_type>(), g);
  graph_paradigm(mbm_map_wf(), update_func_t(), g, k);
  return map_reduce(mbm_final_wf<vd_type>(), stapl::plus<size_t>(), g) / 2;
}

} // namespace stapl

#endif

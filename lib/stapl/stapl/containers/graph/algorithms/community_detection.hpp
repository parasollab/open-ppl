/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_COMMUNITY_DETECTION_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_COMMUNITY_DETECTION_HPP

#include <stapl/containers/graph/algorithms/execute.hpp>

namespace stapl {

namespace community_detection_impl {


//////////////////////////////////////////////////////////////////////
/// @brief Function to compute the most frequent label occurring in a
/// set of labels.
///
/// @param labels The comtainer of labels.
/// @param vd Any initial label.
/// @return The most frequent label.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template<typename C, typename VD>
VD compute_most_frequent_label(C const& labels, VD const& vd)
{
  std::map<VD, size_t> counts;
  auto min_label = vd;
  for (auto const& e : labels) {
    counts[e]++;
    if (e < vd)
      min_label = e;
  }
  size_t max_count = 1;
  VD most_frequent = min_label;
  for (auto const& x : counts)
    if (max_count < x.second) {
      max_count = x.second;
      most_frequent = x.first;
    }

  return most_frequent;
}


//////////////////////////////////////////////////////////////////////
/// @brief Reducer functor for @ref community_detection().
///
/// Reduces two community detection properties to update the first one.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct vp_reducer
{
  template<typename VP1, typename VP2>
  void operator()(VP1& p1, VP2& p2) const
  {
    std::stringstream ss;
    for (auto const& e : p2.label_vector())
      p1.label_vector_add(e);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Functor to add the value of the incomming label to the
/// target vertex's auxiliary labels.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct update_func
{
  typedef bool result_type;
  typedef size_t vd_type;

  vd_type m_label;

  update_func(vd_type const& label = std::numeric_limits<vd_type>::max())
    : m_label(label)
  { }

  template <class Vertex>
  bool operator()(Vertex&& target) const
  {
    target.property().label_vector_add(m_label);
    return true;
  }

  void define_type(typer& t)
  { t.member(m_label); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function to compute the label of a vertex and push it
/// to the vertex's neighbors.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct community_detection_wf
{
  size_t m_max_iter;

  community_detection_wf(size_t max_iter)
    : m_max_iter(max_iter)
  { }

  typedef bool result_type;

  template<typename Vertex, typename GraphVisitor>
  bool operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    if (graph_visitor.level() == 1) {
      v.property().label(v.descriptor());
    }
    if (graph_visitor.level() % 2 == 0) {
      auto most_frequent_label
        = compute_most_frequent_label(v.property().label_vector(),
                                      v.descriptor());

      v.property().label_vector_clear();
      bool active;
      if (most_frequent_label == v.property().label()) {
        active = false;
      } else {
        v.property().label(most_frequent_label);
        active = true;
      }
      return active && graph_visitor.level() <= m_max_iter;
    } else if (graph_visitor.level() % 2 == 1) {
      graph_visitor.visit_all_edges(std::forward<Vertex>(v),
                                    update_func(v.property().label()));
      // return true if the label was changed, and the iterations are
      // less than the maximum allowed.
      return true;
    }
    return true;
  }

  void define_type(typer& t)
  { t.member(m_max_iter); }
};

}; // namespace community_detection_impl;


//////////////////////////////////////////////////////////////////////
/// @brief Parallel Community Detection Algorithm based on
/// modularity maximization.
///
/// Performs a variant of label-propagation on the input @ref graph_view,
/// to find the most frequent label in each vertex's neighborhood, which
/// is set as its community on its property.
/// @policy a policy for execution
/// @param graph The @ref graph_view over the input graph.
/// @param max_iter The maximum number of iterations allowed to execute.
/// @return The number of iterations performed.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Policy, typename GView>
size_t community_detection(Policy&& policy, GView& graph, size_t max_iter = 20)
{
  using namespace community_detection_impl;
  using neighbor_op = update_func;
  using vertex_op = community_detection_wf;

  sgl::execute(
      std::forward<Policy>(policy), graph, vertex_op{max_iter},
      neighbor_op{}, vp_reducer{}
  );

  return max_iter;
}

} // namespace stapl
#endif

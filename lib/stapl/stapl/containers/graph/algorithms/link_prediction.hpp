/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/


#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_LINK_PREDICTION_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_LINK_PREDICTION_HPP

#include <stapl/containers/graph/algorithms/paradigms/graph_paradigm.hpp>

namespace stapl {

namespace lp_algo_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor #1 for @ref link_prediction().
///
/// Adds the initiator of the potential edge to the target of the
/// potential edge's buffer.
/// @tparam VD Type of the vertex-descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VD>
class update_func1
{
public:
  typedef VD        parent_type;
  parent_type       m_parent;

  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param p The initiator (source) of the potential edge.
  //////////////////////////////////////////////////////////////////////
  update_func1(parent_type p = 0)
    : m_parent(p)
  { }

  template <class Vertex>
  result_type operator()(Vertex&& target) const
  {
    target.property().step_2_buffer_add(m_parent);
    return true;
  }

  void define_type(typer& t)
  { t.member(m_parent); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor #2 for @ref link_prediction().
///
/// Propagates the initiator of the potential edge to the potential
/// common neighbors with potential edge's target.
/// @tparam VD Type of the vertex-descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VD>
class update_func2
{
public:
  typedef VD               parent_type;
  parent_type              m_potential_target;
  std::vector<parent_type> m_potential_sources;

  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param p The target of the potential edge.
  /// @param potential_sources The list of initiators (sources) of the
  /// potential edge.
  //////////////////////////////////////////////////////////////////////
  update_func2(parent_type p,
               std::vector<parent_type> const& potential_sources)
    : m_potential_target(p), m_potential_sources(potential_sources)
  { }

  update_func2(void) = default;

  template <class Vertex>
  result_type operator()(Vertex&& target) const
  {
    target.property().step_3_buffer_add(m_potential_target,
                                        m_potential_sources);
    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_potential_target);
    t.member(m_potential_sources);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor #3 for @ref link_prediction().
///
/// Propagates the value from common-neighbors of the source & target
/// of the potential edge to the source of the potential edge and
/// add it to the current probability for that edge on the source vertex.
/// @tparam VD Type of the vertex-descriptor.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <typename VD>
class update_func3
{
public:
  typedef VD   parent_type;
  parent_type  m_potential_target;
  double       m_log_degree_of_common_neighbor;

  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param p The target of the potential edge.
  /// @param log_degree The logarithm of the inverse of the degree of
  /// a common neighbor of the potential edge.
  /// potential edge.
  //////////////////////////////////////////////////////////////////////
  update_func3(parent_type const& p = 0, double const& d = 0)
    : m_potential_target(p), m_log_degree_of_common_neighbor(d)
  { }

  template <class Vertex>
  result_type operator()(Vertex&& target) const
  {
    target.property().add_link_probability(m_potential_target,
                                           m_log_degree_of_common_neighbor);
    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_potential_target);
    t.member(m_log_degree_of_common_neighbor);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function for @ref link_prediction().
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct lpaa_map_wf
{
  size_t m_size;

  lpaa_map_wf(size_t sz)
    : m_size(sz)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Finds an edge with the specified target.
  //////////////////////////////////////////////////////////////////////
  template <class VD>
  struct eq_target
  {
    VD m_vd;

    eq_target(VD const& v)
      : m_vd(v)
    { }

    template <class Edge>
    bool operator()(Edge const& e) const
    { return e.target() == m_vd;  }
  };

  typedef bool result_type;

  template<class Vertex, class GraphVisitor>
  result_type operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    using descriptor_t = typename std::decay<Vertex>::type::vertex_descriptor;

    if (graph_visitor.level() == 1) {  // first SS to initiate prediction.
      const size_t lp_sz = m_size;
      v.property().link_probability_size(lp_sz);
      for (size_t i=0; i<lp_sz; ++i)
        v.property().link_probability(i, 0);
      for (auto const& e : v)
        v.property().link_probability(e.target(), 1);
      for (size_t i=0; i<lp_sz; ++i)
        if (v.property().link_probability(i) != 1) {
           graph_visitor.visit(i, update_func1<descriptor_t>(v.descriptor()));
        }
      return true;
    } else if (graph_visitor.level() == 2) {  // find common neighbors.
      graph_visitor.visit_all_edges(v,
        update_func2<descriptor_t>(v.descriptor(),
                                   v.property().step_2_buffer()));
      v.property().step_2_buffer_clear();
      return true;
    } else if (graph_visitor.level() == 3) {  // complete the circuit.
      const double log_degree = 1.0/log(double(v.size()));
      const auto initiators = v.property().step_3_buffer();
      v.property().step_3_buffer_clear();
      for (auto const& m : initiators) {
        const auto y = m.first;
        for (auto const& x : m.second) {
          if (std::find_if(v.begin(), v.end(), eq_target<descriptor_t>(x))
              != v.end())
            graph_visitor.visit(x, update_func3<descriptor_t>(y, log_degree));
        }
      }
      return true;
    } else if (graph_visitor.level() == 4) {  // fix self-edge probabilities.
      v.property().link_probability(v.descriptor(), 0);
      return false;
    }
    return false;
  }

  void define_type(typer& t)
  { t.member(m_size); }
};

} // namespace lp_algo_detail


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Adamic-Adar Link-Prediction Algorithm
///
/// Predicts the probability that an edge (that does not exist yet)
/// will be added.
/// The algorithm is based on the Adamic-Adar model, where the probability
/// of an edge (x,y) is given by:
/// AA(x,y) = Sum_{n \in Common-neighbors(x,y)}(1 / log(degree(n)))
/// A value of 1 indicates the edge exists already, and a value of zero is
/// the lowest.
/// @param g The @ref graph_view over the input graph.
/// @param k The maximum amount of asynchrony allowed in each phase.
/// 0 <= k <= inf.
/// k == 0 implies level-synchronous paradigm.
/// k >= D implies fully asynchronous paradigm (D is diameter of graph).
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<class GView>
void link_prediction(GView& g, size_t k = 0)
{
  using namespace lp_algo_detail;
  typedef typename GView::vertex_descriptor vd_type;
  typedef update_func1<vd_type> update_func_t;
  graph_paradigm(lpaa_map_wf(g.size()), update_func_t(), g, k);
}

} // namespace stapl

#endif

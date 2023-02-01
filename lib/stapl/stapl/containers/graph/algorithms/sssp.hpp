/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_SSSP_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_SSSP_HPP

#include <stapl/containers/graph/algorithms/execute.hpp>

namespace stapl {

namespace sssp_algo_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex-initializer functor for @ref sssp().
///
/// Initializes vertices' SSSP-parent, SSSP-distance, and active status.
/// If the vertex is the source, the distance is zero and it is set to active.
/// All other vertices are set to inactive and their distance set to infinity.
/// All vertices' SSSP-parents are set to their own descriptors.
/// @tparam VD Type of the vertex-descriptor.
/// @tparam DistType Type of the vertex-distance.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <class VD, class DistType>
class sssp_init_wf
{
  typedef VD vd_type;
  vd_type    m_source;

public:
  typedef void result_type;

  sssp_init_wf(vd_type source)
    : m_source(source)
  { }

  template <class Vertex>
  void operator()(Vertex v) const
  {
    if (v.descriptor() == m_source) {
      v.property().set_active(true);
      v.property().distance(DistType(0));
    } else {
      v.property().set_active(false);
      v.property().distance(std::numeric_limits<DistType>::max());
    }
    v.property().parent(v.descriptor());
  }

  void define_type(typer& t)
  { t.member(m_source); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor @ref sssp().
///
/// Updates the target vertex with SSSP-parent and SSSP-distance
/// information, if the target vertex has not been visited before, or
/// if the target's current distance is greater than the incoming distance.
/// @tparam VD Type of the vertex-descriptor.
/// @tparam DistType Type of the vertex-distance.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <class VD, class DistType>
class update_func
{
public:
  typedef VD        parent_type;
  typedef DistType  dist_type;

  parent_type       m_parent;
  dist_type         m_dist;

  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param p The SSSP-parent of the target vertex.
  //////////////////////////////////////////////////////////////////////
  update_func(parent_type const& p = 0,
              dist_type const& d = std::numeric_limits<dist_type>::max())
    : m_parent(p), m_dist(d)
  { }

  template <class Vertex>
  bool operator()(Vertex&& target) const
  {
    if (m_dist < target.property().distance()) {
      target.property().distance(m_dist);
      target.property().parent(m_parent);
      target.property().set_active(true);
      return true;
    }
    //else ignore.
    return false;
  }

  void set_distance(dist_type const& dist)
  { m_dist = dist; }

  void define_type(typer& t)
  {
    t.member(m_parent);
    t.member(m_dist);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function for @ref sssp().
///
/// A vertex is visited if it is active. Active vertices update their
/// neighbors with new SSSP distance and parent information.
/// Returns true if vertex was active, false otherwise.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct sssp_wf
{
  typedef bool result_type;

  template<typename Vertex, typename GraphVisitor>
  bool operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    using vertex_descriptor =
      typename std::decay<Vertex>::type::vertex_descriptor;

    if (v.property().is_active()) {
      update_func<vertex_descriptor, double>
        uf(v.descriptor());

      size_t dist = v.property().distance();
      typename Vertex::adj_edge_iterator it = v.begin(), it_e = v.end();
      for (; it != it_e; ++it) {
        uf.set_distance(dist + (*it).property());
        graph_visitor.visit((*it).target(), uf);
      }
      v.property().set_active(false);
      return true;
    }
    return false;
  }
};

} //namespace sssp_algo_detail

//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Single-Source Shortest Paths (SSSP) algorithm.
///
/// Performs a single-source shortest path query on the input @ref graph_view,
/// storing the shortest path distance and parent on each reachable vertex.
/// All vertices will be initialized with their distance as infinity (MAX)
/// and their active state as false, except the source-vertex, which will
/// have its distance set to zero (0) and active state set to true.
/// Parents of each vertex will be initialized to the vertex's descriptor.
/// @param g The @ref graph_view over the input graph.
/// @param source The descriptor of the source vertex for this traversal.
/// @param k The maximum amount of asynchrony allowed in each phase.
/// 0 <= k <= inf.
/// k == 0 implies level-synchronous SSSP.
/// k >= D implies fully asynchronous SSSP (D is diameter of graph).
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename GView>
size_t sssp(GView& g, typename GView::vertex_descriptor const& source,
            size_t k=0)
{
  using namespace sssp_algo_detail;
  typedef typename GView::vertex_descriptor vd_type;

  // Initialize the vertices.
  map_func(sssp_init_wf<vd_type, double>(source), g);

  typedef update_func<vd_type, double> update_func_t;
  return kla_paradigm(sssp_wf{}, update_func_t{}, g, k);
}

} //namespace stapl

#endif

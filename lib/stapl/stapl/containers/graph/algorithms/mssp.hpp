/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_MSSP_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_MSSP_HPP

#include <stapl/containers/graph/algorithms/paradigms/graph_paradigm.hpp>

namespace stapl {

namespace mssp_algo_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex-initializer functor for @ref mssp().
///
/// Initializes vertices' MSSP-parents, MSSP-distances, and active status.
/// If the vertex is one of the sources, the distance for its traversal-ID is
/// zero and it is set to active w.r.t. its own traversal-ID.
/// Non-source vertices are set to inactive and their distance set to infinity.
/// All vertices' SSSP-parents are set to their own descriptors.
/// @tparam VD Type of the vertex-descriptor.
/// @tparam DistType Type of the vertex-distance.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <class VD, class DistType>
class mssp_init_wf
{
  typedef VD vd_type;
  typedef std::vector<vd_type> sources_t;
  sources_t m_sources;

public:
  typedef void result_type;

  mssp_init_wf(sources_t const& sources)
    : m_sources(sources)
  { }

  template <class Vertex>
  void operator()(Vertex v)
  {
    typename sources_t::iterator it =
      std::find(m_sources.begin(), m_sources.end(), v.descriptor());
    for (size_t i=0; i<m_sources.size(); ++i) {
      v.property().distance(m_sources[i], std::numeric_limits<DistType>::max());
      v.property().parent(m_sources[i], v.descriptor());
    }
    if (it != m_sources.end()) {
      v.property().set_active(v.descriptor());
      v.property().distance(v.descriptor(), DistType(0));
    }
  }

  void define_type(typer& t)
  { t.member(m_sources); }
};


//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor @ref mssp().
///
/// Updates the target vertex with MSSP-parent and MSSP-distance
/// information from a particular traversal-ID, if the target vertex has
/// not been visited before, or if the target's current distance is
/// greater than the incoming distance.
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

  parent_type       m_traversal_id;
  parent_type       m_parent;
  dist_type         m_dist;

  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param p The MSSP-parent of the target vertex for this traversal.
  /// @param source The ID of this traversal.
  /// @note distance can be set in set_distance(), as it may be different
  /// for each neighbor, while source and parent are same for
  /// all neighbors.
  //////////////////////////////////////////////////////////////////////
  update_func(parent_type const& source = 0, parent_type const& p = 0,
              dist_type const& d = std::numeric_limits<dist_type>::max())
    : m_traversal_id(source), m_parent(p), m_dist(d)
  { }

  template <class Vertex>
  bool operator()(Vertex&& target) const
  {
    if (m_dist < target.property().distance(m_traversal_id)) {
      target.property().distance(m_traversal_id, m_dist);
      target.property().parent(m_traversal_id, m_parent);
      target.property().set_active(m_traversal_id);
      return true;
    }
    //else ignore.
    return false;
  }

  void set_distance(dist_type const& dist)
  { m_dist = dist; }

  void define_type(typer& t)
  {
    t.member(m_traversal_id);
    t.member(m_parent);
    t.member(m_dist);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function for @ref mssp().
///
/// A vertex is visited if it is active. Active vertices update their
/// neighbors with new MSSP distance and parent information for the given
/// traversal-ID.
/// Returns true if vertex was active, false otherwise.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct mssp_wf
{
  using concurrency_model = sgl::strong_concurrency;

  template<typename Vertex, typename GraphVisitor>
  bool operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    using vertex_descriptor =
      typename std::decay<Vertex>::type::vertex_descriptor;

    size_t active_traversal_id;
    bool was_active = false;
    while (v.property().is_active()) {
      was_active = true;
      active_traversal_id = v.property().next_active_traversal();
      update_func<vertex_descriptor, double>
        uf(active_traversal_id, v.descriptor());

      double dist = v.property().distance(active_traversal_id);

      for (auto it = v.begin(), it_e = v.end(); it != it_e; ++it) {
        uf.set_distance(dist + (*it).property());
        graph_visitor.visit((*it).target(), uf);
      }
      v.property().set_inactive(active_traversal_id);
    }

    if (was_active)
      return true;

    return false;
  }
};

} //namespace mssp_algo_detail


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Multi-Source Shortest Paths (MSSP) algorithm.
///
/// Performs a multi-source shortest path query on the input @ref graph_view,
/// storing the shortest path distances and parents on each reachable vertex
/// for each traversal.
/// All vertices will be initialized with their distances as infinity (MAX)
/// and their active states as empty, except the source-vertex for each
/// traversal, which will have its distance set to zero (0) and active state
/// set to true for that traversal.
/// Parents of each vertex will be initialized to the vertex's descriptor.
/// @param g The @ref graph_view over the input graph.
/// @param sources The descriptors of the source vertices for this traversal.
/// @param k The maximum amount of asynchrony allowed in each phase.
/// 0 <= k <= inf.
/// k == 0 implies level-synchronous MSSP.
/// k >= D implies fully asynchronous MSSP (D is diameter of graph).
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<class GView>
size_t mssp(GView& g,
            std::vector<typename GView::vertex_descriptor> const& sources,
            size_t k=0)
{
  using namespace mssp_algo_detail;
  typedef typename GView::vertex_descriptor vd_type;

  // Initialize the vertices.
  map_func(mssp_init_wf<vd_type, double>(sources), g);

  typedef update_func<vd_type, double> update_func_t;
  return graph_paradigm(mssp_wf(), update_func_t(), g, k);
}

} //namespace stapl

#endif

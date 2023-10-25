/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

//STAPL includes
#ifndef STAPL_ALGORITHMS_BETWEENNESS_CENTRALITY_HPP
#define STAPL_ALGORITHMS_BETWEENNESS_CENTRALITY_HPP

#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/containers/graph/algorithms/execute.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <cmath>
#include <vector>

namespace stapl {

namespace bc_algo_detail {

typedef typename properties::bc_property::vd_type       vd_type;
typedef typename properties::bc_property::dist_type     dist_type;
typedef typename properties::bc_property::sigma_type    sigma_type;
typedef typename properties::bc_property::delta_type    delta_type;
typedef typename properties::bc_property::bfs_dag_type  bfs_dag_type;
typedef typename properties::bc_property::level_type    level_type;
typedef typename properties::bc_property::active_t      active_t;

//////////////////////////////////////////////////////////////////////
/// @brief Post executing workfunction called at the end of each level.
/// Pushes pending sources to the list of active sources for a vertex.
/// @see properties::bc_property::push_pending()
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct post_execute
{
  struct post_execute_wf
  {
    typedef void result_type;

    template<typename Vertex>
    void operator()(Vertex v) const
    { v.property().push_pending(); }
  };

  template<typename GView>
  void operator()(GView gv, size_t iterations) const
  {
    map_func(post_execute_wf(), gv);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Update function for @ref bc_mssp_wf. Calculates the sigma values
/// for active sources.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class update_func
{
public:
  vd_type     m_source_id;
  vd_type     m_parent;
  dist_type   m_dist;
  sigma_type  m_sigma;

  typedef bool result_type;

  update_func(vd_type source, vd_type parent, dist_type dist, sigma_type sig)
    : m_source_id(source), m_parent(parent), m_dist(dist), m_sigma(sig)
  { }

  update_func() = default;

  template <typename Vertex>
  bool operator()(Vertex&& target) const
  {
    if (target.property().distance(m_source_id) == 0) {
      target.property().set_pending(m_source_id);
      target.property().distance(m_source_id, m_dist);
    }
    if (target.property().distance(m_source_id) == m_dist) {
      target.property().bfs_dag(m_source_id, m_parent, m_sigma);
    }
    return true;
  }

  void define_type(typer& t)
  {
    t.member(m_source_id);
    t.member(m_parent);
    t.member(m_dist);
    t.member(m_sigma);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Workfunction to calculate the sigma values for active sources
/// and builds bfs dags.
/// @see update_func
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct bc_mssp_wf
{
  typedef bool result_type;

  template<typename Vertex, typename GraphVisitor>
  bool operator()(Vertex&& v_vert, GraphVisitor&& graph_visitor) const
  {
    vd_type v = v_vert.descriptor();
    bool was_active = v_vert.property().is_active();
    while (v_vert.property().is_active()) {
      vd_type source_id = v_vert.property().next_active_traversal();
      update_func uf(source_id, v, v_vert.property().distance(source_id)+1,
                                   v_vert.property().sigma(source_id));
      graph_visitor.visit_all_edges(v_vert, uf);
    }
    return was_active;
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Workfunction to calculate delta
/// @see dependency_wf
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct apply_delta_wf
{
  typedef void result_type;

  size_t m_source_id;
  delta_type m_sigma;
  delta_type m_delta;

  apply_delta_wf(size_t source_id, delta_type sigma, delta_type delta)
    : m_source_id(source_id), m_sigma(sigma), m_delta(delta)
  { }

  template <typename Property>
  void operator()(Property& target) const
  {
    target.delta(m_source_id, target.delta(m_source_id)
                   + (target.sigma(m_source_id) / m_sigma) * (1.0 + m_delta));
  }

  void define_type(typer& t)
  {
    t.member(m_source_id);
    t.member(m_sigma);
    t.member(m_delta);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief "Reverse" BFS that calculates betweenness centrality
/// value of each vertex using sigma and delta values of current
/// and child vertices.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct dependency_wf
{
  typedef void result_type;

  vd_type m_source_offset;
  dist_type m_level;

  dependency_wf(vd_type source_offset,  dist_type curr_level)
    : m_source_offset(source_offset), m_level(curr_level)
  { }

  template <typename Vertex, typename GView>
  void operator()(Vertex v_vert, GView& gv)
  {
    size_t v = v_vert.descriptor();
    const level_type sources = v_vert.property().level_sources(m_level);

    for (size_t s = 0; s < sources.size(); ++s) {
      const bfs_dag_type bfs_dag = v_vert.property().bfs_dag(sources[s]);
      if (!bfs_dag.empty()) {
        sigma_type ext_sigma = v_vert.property().sigma(sources[s]);
        delta_type ext_delta = v_vert.property().delta(sources[s]);
        apply_delta_wf wf(sources[s], ext_sigma, ext_delta);
        for (size_t i = 0; i < bfs_dag.size(); ++i) {
          gv.vp_apply_async(bfs_dag[i], wf);
        }
        if (v != (sources[s]+ m_source_offset)) {
          v_vert.property().BC(v_vert.property().BC()+ext_delta);
        }
      }
    }
  }

  void define_type(typer& t)
  {
    t.member(m_source_offset);
    t.member(m_level);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Initializes vertex properties and allocates memory for
/// the number of sources to calculate per iteration.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class map_init
{
  size_t m_num_sources;

public:
  typedef void result_type;

  map_init(size_t num_sources)
    : m_num_sources(num_sources)
  { }

  template <class Vertex>
  void operator()(Vertex v_vert)
  { v_vert.property().initialize(m_num_sources); }

  void define_type(typer& t)
  { t.member(m_num_sources); }
};

//////////////////////////////////////////////////////////////////////
/// @brief Resets the calculations made for previous sources and
/// sets sources in the range [begin, end) as active.
/// @todo Explore activation of non-contiguous ranges of vertices.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
class map_resources
{
  size_t m_begin;
  size_t m_end;

public:
  typedef void result_type;

  map_resources(vd_type begin, vd_type end)
    : m_begin(begin), m_end(end)
  { }

  template <typename Vertex>
  void operator()(Vertex v_vert)
  {
    vd_type v = v_vert.descriptor();
    v_vert.property().reset_sources();
    if (v >= m_begin && v < m_end) {
      v_vert.property().distance(v - m_begin, 1);
      v_vert.property().sigma(v - m_begin, 1);
      v_vert.property().set_active(v - m_begin);
    }
  }

  void define_type(typer& t)
  {
    t.member(m_begin);
    t.member(m_end);
  }
};

} // namespace bc_algo_detail

//////////////////////////////////////////////////////////////////////
/// @brief Computes betweenness centrality value of unweighted,
/// directed graph.
/// @param policy A policy for execution.
/// @param gv View of graph on which centrality will be computed.
/// @param num_sources Number of sources to process at a time, if
/// num_sources equals zero then all sources are processed at the
/// the same time.
/// @param partial Indicating whether or not all vertices should be
/// sources. If not, only num_sources traversals are considered
/// @todo Explore activation of non-contiguous ranges of vertices.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<typename Policy, typename GView>
void betweenness_centrality(Policy&& policy, GView& gv, size_t num_sources=0,
                            bool partial=false)
{
  using namespace bc_algo_detail;

  const size_t gv_num_verts = gv.num_vertices();

  if (num_sources > gv_num_verts || num_sources == 0)
    num_sources = gv_num_verts;

  // Initialize the vertices.
  map_func(map_init(num_sources), gv);

  auto finish_pred = kla_detail::false_predicate{};

  for (size_t s = 0; s < gv_num_verts; s += num_sources) {
    map_func(map_resources(s, s+num_sources), gv);

    dist_type level = sgl::execute(
      std::forward<Policy>(policy), gv, bc_mssp_wf(), update_func(),
      sgl::noop_vp_reducer{}, finish_pred, post_execute{});

    for (; level > 0; --level) {
      map_func(dependency_wf(s, level), gv, make_repeat_view(gv));
    }

    if (partial)
      break;
  }
}

//////////////////////////////////////////////////////////////////////
/// @brief Computes betweenness centrality value of unweighted,
/// directed graph.
/// @param gv View of graph on which centrality will be computed.
/// @param num_sources Number of sources to process at a time, if
/// num_sources equals zero then all sources are processed at the
/// the same time.
/// @param partial Indicating whether or not all vertices should be
/// sources. If not, only num_sources traversals are considered
/// @todo Explore activation of non-contiguous ranges of vertices.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template <typename GView>
void betweenness_centrality(GView& gv, size_t num_sources=0, bool partial=false)
{
  return betweenness_centrality(stapl::sgl::make_execution_policy("lsync", gv),
                                gv, num_sources, partial);
}

} // namespace stapl

#endif

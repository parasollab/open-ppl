/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_SWEEP_H_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_SWEEP_H_HPP

#include <algorithm>
#include <benchmarks/sweep/sweep.hpp>
//#include <stapl/containers/graph/algorithms/paradigms/graph_paradigm.hpp>
#include "paragraph_based_kla/graph_paradigm.hpp"

namespace stapl {

template<class GViewTop, class GView, class InitPropWf, class UpdatePropWf,
         class UpdateCondWf, class FilterEdgeWf, class DirectionType>
size_t sweep_recursive(GViewTop& top_graph, std::vector<GView*>& lower_graphs,
                       std::vector<DirectionType> const& directions,
                       InitPropWf const& init_prop_wf,
                       UpdatePropWf const& update_prop_wf,
                       UpdateCondWf const& update_cond_wf,
                       FilterEdgeWf const& filter_edge_wf,
                       size_t k=0);

namespace sweep_user_wf {

struct update_h_property;
struct filter_h_edge;

size_t m_x_k = 0;

//////////////////////////////////////////////////////////////////////
/// @brief Computes the dot-product of two directional vectors of the
/// same cardinality
//////////////////////////////////////////////////////////////////////
template<typename Direction1, typename Direction2>
float dot_product(Direction1 const& d1, Direction2 const& d2)
{
  float scalar_product = 0.0;
  if (d1.size() != d2.size())
    abort("Invalid cardinality for direction\n");
  for (size_t i=0; i<d1.size(); ++i)
    scalar_product += d1[i]*d2[i];
  return scalar_product;
}

//////////////////////////////////////////////////////////////////////
/// @brief Indicates whether or not an edge should be visited for the given
/// traversal.
//////////////////////////////////////////////////////////////////////
struct filter_h_edge
{
  template<typename SweepId, typename Direction, typename Edge>
  bool operator() (SweepId const& traversal_id, Direction const& direction,
                   Edge const& e) const
  {
    // Proceed only when the dot-product of edge direction vector and sweep
    // direction vector is greater than zero.
    if (e.property().property.get_cross_edge())
      return false;
    Direction edge_direction(e.property().property.get_direction());
    return (sweep_user_wf::dot_product(edge_direction, direction) > 0);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Computes and updates vertex-property for a sweep, based on
/// incoming values from neighbors. Also returns the value to propagate
/// for current sweep.
//////////////////////////////////////////////////////////////////////
struct update_h_property
{
  struct s1
  {
    typedef size_t result_type;
    template<typename V>
    size_t operator()(V&& v)
    {
      return v.descriptor();
    }
  };

  template<typename SweepId, typename Direction,
           typename ValueContainerType, typename Property, typename GView>
  typename ValueContainerType::value_type operator() (
      SweepId const& traversal_id, Direction const& direction,
      ValueContainerType const& incoming_values,
      Property&& p, std::vector<GView*> g_lower) const
  {
    auto min = *std::min_element(incoming_values.begin(),
                                 incoming_values.end());
    p.property.set_value(traversal_id, direction, min);

    if (g_lower.size() > 0 && !p.children.empty()) {
      // create a domain over children.
      using dom_t = domset1D<size_t>;
      using child_view_type = graph_view<typename GView::view_container_type,
                                         dom_t>;
      dom_t d;
      for (auto const& e : p.children)
        d += e;
      child_view_type children_vw((g_lower.back())->container(), d);

      // Perform a hierarchical sweep on the lower-level graph.
      std::vector<GView*> new_lower_graphs{g_lower.begin(), g_lower.end()-1};
      sweep_recursive(children_vw, new_lower_graphs,
                      std::vector<Direction>{direction},
                      init_property(),
                      update_h_property(),
                      update_condition(),
                      filter_h_edge(), m_x_k);
    }
    return min + 1;
  }
};

} //namespace sweep_user_wf

namespace sweep_algo_detail {

//////////////////////////////////////////////////////////////////////
/// @brief Work-function to initialize edges for @ref sweep().
///
/// Initializes all edges of a graph as non-cross edges.
//////////////////////////////////////////////////////////////////////
struct init_lower_edges
{
  template<typename Vertex>
  void operator()(Vertex&& v) const
  {
    for (auto e : v) {
      e.property().property.set_cross_edge(false);
    }
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Work-function to initialize boundary edges for @ref sweep().
///
/// Marks child edges of a hierarchical edge in the upper-level graph
/// as cross-edges in the lower graph (@ref g_lower). The edges of the
/// lower-level graph must already have been initialized by calling
/// @ref init_lower_edges.
//////////////////////////////////////////////////////////////////////
template <class GLower>
struct init_h_edges
{
  struct init_boundary_edges
  {
    typedef void result_type;
    template<typename Ep>
    void operator()(Ep&& ep) const
    {
      ep.property.set_cross_edge(true);
    }
  };

  init_h_edges(GLower* g_lower)
    : m_g_lower(g_lower)
  {}

  template<typename Vertex>
  void operator()(Vertex&& v)
  {
    for (auto const& e : v) {
      std::vector<typename GLower::edge_descriptor>
        c_edges(e.property().children);
      for (typename GLower::edge_descriptor c : c_edges) {
        m_g_lower->ep_apply_async(c, init_boundary_edges());
      }
    }
  }

  void define_type(typer& t)
  { t.member(m_g_lower); }

  GLower* m_g_lower;
};


//////////////////////////////////////////////////////////////////////
/// @brief Vertex-initializer functor for @ref sweep().
///
/// Initializes vertices' SWEEP-parents, SWEEP-distances, and active status.
/// If the vertex is one of the sources, the distance for its traversal-ID is
/// zero and it is set to active w.r.t. its own traversal-ID.
/// Non-source vertices are set to inactive and their distance set to infinity.
/// All vertices' SSSP-parents are set to their own descriptors.
/// @tparam SweepId Type of the vertex-descriptor.
/// @tparam DistType Type of the vertex-distance.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <class SweepId, class Direction, class ValueType,
          class UserInitPropertyWf>
class sweep_init_h_wf
{
  typedef SweepId vd_type;
  typedef std::vector<Direction> directions_t;
  typedef std::vector<vd_type> sources_t;
  sources_t m_sources;
  directions_t m_directions;
  UserInitPropertyWf m_init_property_wf;
  bool m_mode;

public:
  typedef sources_t result_type;

  sweep_init_h_wf(sources_t const& sources,
                  directions_t const& directions,
                  UserInitPropertyWf const& init_property_wf,
                  bool mode)
    : m_sources(sources), m_directions(directions),
      m_init_property_wf(init_property_wf), m_mode(mode)
  { }

  template <class Vertex>
  result_type operator()(Vertex&& v)
  {
    bool was_active = false;
    if (m_mode) {
      // For each direction, initialize the vertex's property.
      for (size_t i = 0; i < m_directions.size(); ++i) {
        auto const& direction = m_directions[i];
        v.property().property.set_num_incoming_edges_for_direction(direction,
                                                                   0);

        for (auto const& e : v) {
          Direction edge_direction(e.property().property.get_direction());
          auto dot_product = sweep_user_wf::dot_product(edge_direction,
                                                        direction);

          if (e.target() != v.descriptor()) {
            if (dot_product < 0) {
              v.property().property
                .incr_num_incoming_edges_for_direction(direction);
            }
          }
        }

        if (v.property().property
              .get_num_incoming_edges_for_direction(direction) == 0) {
          // Use the index of the direction as the SweepId.
          v.property().property.incoming_sweeps_add(i, direction, ValueType(0));
          v.property().property.set_value(i, direction, ValueType(0));
          was_active = true;
        }
      }
    } else {
      // Initialize the vertex's property for other sources.
      m_init_property_wf(m_sources, m_directions, v.property().property);
    }

    if (was_active) {
      return sources_t{v.descriptor()};
    } else {
      return sources_t{};
    }
  }

  void define_type(typer& t)
  {
    t.member(m_sources);
    t.member(m_directions);
    t.member(m_init_property_wf);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Visitor functor @ref sweep().
///
/// Updates the target vertex with SWEEP-parent and SWEEP-distance
/// information from a particular traversal-ID, if the target vertex has
/// not been visited before, or if the target's current distance is
/// greater than the incoming distance.
/// @tparam SweepId Type of the vertex-descriptor.
/// @tparam DistType Type of the vertex-distance.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <class SweepId, class Direction, class ValueType,
          class UpdateConditionWf>
class update_h_func
{
public:
  typedef SweepId   parent_type;
  typedef Direction direction_type;
  typedef ValueType value_type;

  UpdateConditionWf m_update_condition;
  parent_type       m_traversal_id;
  direction_type    m_direction;
  value_type        m_value;

  typedef bool result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param p The SWEEP-parent of the target vertex for this traversal.
  /// @param source The ID of this traversal.
  /// @note distance can be set in set_distance(), as it may be different
  /// for each neighbor, while source and parent are same for
  /// all neighbors.
  //////////////////////////////////////////////////////////////////////
  update_h_func(UpdateConditionWf const& update_condition = UpdateConditionWf(),
                parent_type const& traversal_id = parent_type(),
                direction_type const& direction = direction_type(),
                value_type const& d = std::numeric_limits<value_type>::max())
    : m_update_condition(update_condition),
      m_traversal_id(traversal_id),
      m_direction(direction),
      m_value(d)
  { }

  template <class Vertex>
  bool operator()(Vertex&& target) const
  {
    // If incoming value for given traversal-id, in the given direction
    // warrants updating the vertex, then add this sweep to the vertex.
    if (m_update_condition(m_traversal_id, m_direction, m_value,
                           target.property().property)) {
      target.property().property.incoming_sweeps_add(m_traversal_id,
                                                     m_direction,
                                                     m_value);
      // A vertex shouldn't be processed until the values for all incoming edges
      // for a given sweep have arrived.
      if (target.property()
            .property.get_num_incoming_edges_for_direction(m_direction)
          == target.property().property.incoming_sweeps_size(m_traversal_id,
                                                             m_direction))
        return true;
    }
    //else ignore.
    return false;
  }

  void set_value(value_type const& d)
  { m_value = d; }

  void define_type(typer& t)
  {
    t.member(m_update_condition);
    t.member(m_traversal_id);
    t.member(m_direction);
    t.member(m_value);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work-function for @ref sweep().
///
/// A vertex is visited if it is active. Active vertices update their
/// neighbors with new SWEEP distance and parent information for the given
/// traversal-ID.
/// Returns true if vertex was active, false otherwise.
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
template <class UpdatePropWf, class UpdateCondWf, class FilterEdgeWf,
          class DirectionType, class ValueType, class GLower>
struct sweep_h_wf
{
  sweep_h_wf(UpdatePropWf const& update_prop_wf,
             UpdateCondWf const& update_cond_wf,
             FilterEdgeWf const& filter_edge_wf,
             std::vector<GLower*>& g_lower)
    : m_update_prop_wf(update_prop_wf),
      m_update_cond_wf(update_cond_wf),
      m_filter_edge_wf(filter_edge_wf),
      m_g_lower(g_lower)
  {}

  typedef bool result_type;

  template<typename Vertex, typename GraphVisitor>
  bool operator()(Vertex v, GraphVisitor graph_visitor) const
  {
    size_t traversal_id;
    bool was_active = false;

    // For each incoming sweep, compute the new value and propagate.
    for (auto const& element : v.property().property.incoming_sweeps()) {
      was_active = true;
      traversal_id = element.first;
      for (auto const& directional_element : element.second) {
        auto direction = directional_element.first;
        // Compute and update the vertex's property for this sweep.
        // Also return the value to propagate for this sweep.
        auto new_sweep_value =
          m_update_prop_wf(traversal_id, direction, directional_element.second,
                           v.property(), m_g_lower);
        update_h_func<typename Vertex::vertex_descriptor,
                      DirectionType, ValueType, UpdateCondWf>
          uf(m_update_cond_wf, traversal_id, direction, new_sweep_value);

        for (typename Vertex::adj_edge_iterator it = v.begin(), it_e = v.end();
             it != it_e; ++it) {
          if (m_filter_edge_wf(traversal_id, direction, *it)) {
            graph_visitor.visit((*it).target(), uf);
          }
        }
      }
    }
    // Reset incoming sweep buffer.
    v.property().property.incoming_sweeps_reset();

    return was_active;
  }

  void define_type(typer& t)
  {
    t.member(m_update_prop_wf);
    t.member(m_update_cond_wf);
    t.member(m_filter_edge_wf);
    t.member(m_g_lower);
  }

  UpdatePropWf m_update_prop_wf;
  UpdateCondWf m_update_cond_wf;
  FilterEdgeWf m_filter_edge_wf;
  std::vector<GLower*> m_g_lower;
};

} //namespace sweep_algo_detail


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Multi-Source Sweep algorithm.
///
/// Performs multi-source sweeps on the input @ref graph_view, starting from
/// identified sources. The sweep from each source happens simultaneously,
/// and is identified by a unqiue sweep-Id, which is the descriptor of the
/// source vertex of that sweep. The vertex-properties are initialized by the
/// given @ref init_prop_wf. The sweep visits each vertex, and computes its new
/// property based on the provided @ref update_prop_wf. The sweep then visits
/// the vertex's neighbors that pass the user-provided edge-flitering based on
/// the @ref filter_edge_wf. If the target neighbor vertex should be updated
/// based on the provided @ref update_cond_wf, it continues the sweep, otherwise
/// the neighbor is not updated.
/// @param top_graph The @ref graph_view over the top-level input graph in
/// hierarchy.
/// @param lower_graphs The rest of the hierarchy, with index-0 being the lowest
/// level.
/// @param sources The descriptors of the source vertices for the sweeps.
/// @param init_prop_wf The work-function to initialize vertex properties.
/// @param update_prop_wf The functor to compute updated vertex-property.
/// @param update_cond_wf The functor to specify if neighbor should be updated.
/// @param filter_edge_wf The functor to specify if neighbor should be visited.
/// @param k The maximum amount of asynchrony allowed in each phase.
/// 0 <= k <= inf.
/// k == 0 implies level-synchronous SWEEP.
/// k >= D implies fully asynchronous SWEEP (D is diameter of graph).
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<class GViewTop, class GView, class InitPropWf, class UpdatePropWf,
         class UpdateCondWf, class FilterEdgeWf, class DirectionType>
size_t sweep_recursive(GViewTop& top_graph, std::vector<GView*>& lower_graphs,
                       std::vector<DirectionType> const& directions,
                       InitPropWf const& init_prop_wf,
                       UpdatePropWf const& update_prop_wf,
                       UpdateCondWf const& update_cond_wf,
                       FilterEdgeWf const& filter_edge_wf,
                       size_t k)
{
  using namespace sweep_algo_detail;
  typedef typename GView::vertex_descriptor vd_type;
  typedef double ValueType;

  sweep_user_wf::m_x_k = k;

  typedef update_h_func<vd_type, DirectionType, ValueType, UpdateCondWf>
    update_func_t;
  auto ret = kla_pg::graph_paradigm_pg(
      sweep_h_wf<UpdatePropWf, UpdateCondWf, FilterEdgeWf, DirectionType,
                 ValueType, GView>(update_prop_wf, update_cond_wf,
                                   filter_edge_wf, lower_graphs),
      update_func_t(update_cond_wf), top_graph, k);

  return ret;
}


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Multi-Source Sweep algorithm.
///
/// Performs multi-source sweeps on the input @ref graph_view, starting from
/// the identified sources. The sweep from each source happens simultaneously,
/// and is identified by a unqiue sweep-Id, which is the descriptor of the
/// source vertex of that sweep. The vertex-properties are initialized by the
/// given @ref init_prop_wf. The sweep visits each vertex, and computes its new
/// property based on the provided @ref update_prop_wf. The sweep then visits
/// the vertex's neighbors that pass the user-provided edge-flitering based on
/// the @ref filter_edge_wf. If the target neighbor vertex should be updated
/// based on the provided @ref update_cond_wf, it continues the sweep, otherwise
/// the neighbor is not updated.
/// @param graphs The vector of @ref graph_views representing a graph hierarchy.
/// graphs[0] should represent the lowest level of the hierarchy, with
/// subsequent indices building upon the i-1th graph.
/// @param sources The descriptors of the source vertices for the sweeps.
/// @param init_prop_wf The work-function to initialize vertex properties.
/// @param update_prop_wf The functor to compute updated vertex-property.
/// @param update_cond_wf The functor to specify if neighbor should be updated.
/// @param filter_edge_wf The functor to specify if neighbor should be visited.
/// @param k The maximum amount of asynchrony allowed in each phase.
/// 0 <= k <= inf.
/// k == 0 implies level-synchronous SWEEP.
/// k >= D implies fully asynchronous SWEEP (D is diameter of graph).
/// @return The number of iterations performed by the paradigm.
/// @ingroup pgraphAlgo
//////////////////////////////////////////////////////////////////////
template<class GView, class InitPropWf, class UpdatePropWf, class UpdateCondWf,
         class FilterEdgeWf, class DirectionType>
size_t sweep(std::vector<GView*>& graphs,
             std::vector<DirectionType> const& directions,
             InitPropWf const& init_prop_wf,
             UpdatePropWf const& update_prop_wf,
             UpdateCondWf const& update_cond_wf,
             FilterEdgeWf const& filter_edge_wf,
             size_t k=0)
{
  using namespace sweep_algo_detail;
  typedef typename GView::vertex_descriptor vd_type;
  typedef double ValueType;

  sweep_user_wf::m_x_k = k;

  counter<default_timer> t;
  t.start();

  double llinit = 0;
  // Initialize edges in lower graph to indicate boundary edges.
  {
    for (int i = graphs.size()-2; i >= 0; i--)
      map_func(init_lower_edges(), *graphs[i]);

    for (int i = graphs.size()-2; i >= 0; i--)
      map_func(init_h_edges<GView>(graphs[i]), *graphs[i+1]);

    llinit += t.stop(); t.reset(); t.start();

    for (int i = graphs.size()-1; i >= 0; --i) {
      // Initialize the vertices and sources. Sources are direction-dependent --
      // a vertex is a source for directions where it has no incoming edges.
      auto new_sources = map_reduce(
        sweep_init_h_wf<vd_type, DirectionType, ValueType, InitPropWf>(
          {}, directions, init_prop_wf, true), join_sources_wf(), *graphs[i]);
      if (new_sources.empty())
        return 0;

      map_func(
        sweep_init_h_wf<vd_type, DirectionType, ValueType, InitPropWf>(
          new_sources, directions, init_prop_wf, false), *graphs[i]);
    }
  }
  double sinit = t.stop(); t.reset(); t.start();

  std::vector<GView*> lower_graphs;
  for (size_t i = 0; i < graphs.size()-1; ++i) {
    lower_graphs.push_back(graphs[i]);
  }

  // Call actual sweep.
  auto ret = sweep_recursive(*graphs.back(), lower_graphs, directions,
                             init_prop_wf, update_prop_wf, update_cond_wf,
                             filter_edge_wf, k);
  double swt = t.stop(); t.reset(); t.start();

  stapl::do_once([&llinit, &sinit, &swt]{
      std::cout << "\n >> sweep times: " << llinit << ", " << sinit << ", "
                << swt << std::endl; });

  return ret;
}

} //namespace stapl

#endif

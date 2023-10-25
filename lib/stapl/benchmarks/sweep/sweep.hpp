/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_SWEEP_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_SWEEP_HPP

#include <algorithm>
#include <stapl/containers/graph/algorithms/paradigms/graph_paradigm.hpp>

namespace stapl {

namespace sweep_user_wf {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex-property initializer functor for @ref sweep().
///
/// Initializes vertex properties to default values.
//////////////////////////////////////////////////////////////////////
struct init_property
{
  template<typename SourcesContainer, typename DirectionContainer,
           typename Property>
  void operator() (SourcesContainer const& sources,
                   DirectionContainer const& directions,
                   Property&& p) const
  {
    for (size_t i = 0; i < directions.size(); ++i) {
      auto const& d = directions[i];
      p.set_value(i, d, std::numeric_limits<size_t>::max());
    }
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Indicates if incoming value for given sweep-id warrants
/// updating vertex.
//////////////////////////////////////////////////////////////////////
struct update_condition
{
  template<typename SweepId, typename Direction, typename ValueType,
           typename Property>
  bool operator()(SweepId const& traversal_id, Direction const& direction,
                  ValueType const& incoming_value,
                  Property const& p) const
  {
    return p.get_value(traversal_id, direction) > incoming_value;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Computes and updates vertex-property for a sweep, based on
/// incoming values from neighbors. Also returns the value to propagate
/// for current sweep.
//////////////////////////////////////////////////////////////////////
struct update_property
{
  template<typename SweepId, typename Direction,
           typename ValueContainerType, typename Property>
  typename ValueContainerType::value_type operator() (
      SweepId const& traversal_id, Direction const& direction,
      ValueContainerType const& incoming_values,
      Property&& p) const
  {
    // TODO(ananvay): hierarchical here.
    auto min = *std::min_element(incoming_values.begin(),
                                 incoming_values.end());
    p.set_value(traversal_id, direction, min);
    return min + 1;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Indicates whether or not an edge should be visited for the given
/// traversal.
//////////////////////////////////////////////////////////////////////
struct filter_edge
{
  template<typename SweepId, typename Direction, typename Edge>
  bool operator() (SweepId const& traversal_id, Direction const& direction,
                   Edge const& e) const
  {
    // Proceed only when the dot-product of edge direction vector and sweep
    // direction vector is greater than zero.
    Direction edge_direction(e.property());
    return (std::inner_product(edge_direction.begin(), edge_direction.end(),
                               direction.begin(), 0) > 0);
  }
};

} //namespace sweep_user_wf

namespace sweep_algo_detail {

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
class sweep_init_wf
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

  sweep_init_wf(sources_t const& sources,
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
        v.property().set_num_incoming_edges_for_direction(direction, 0);

        for (auto const& e : v) {
          Direction edge_direction(e.property());
          auto dot_product = std::inner_product(edge_direction.begin(),
                                                edge_direction.end(),
                                                direction.begin(),
                                                0);

          if (e.target() != v.descriptor()) {
            if (dot_product < 0) {
              v.property().incr_num_incoming_edges_for_direction(direction);
            }
          }
        }

        if (v.property().get_num_incoming_edges_for_direction(direction) == 0) {
          // Use the index of the direction as the SweepId.
          v.property().incoming_sweeps_add(i, direction, ValueType(0));
          v.property().set_value(i, direction, ValueType(0));
          was_active = true;
        }
      }
    } else {
      // Initialize the vertex's property for other sources.
      m_init_property_wf(m_sources, m_directions, v.property());
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
/// @brief Reducer functor for joining sources for @ref sweep().
///
/// @ingroup pgraphAlgoDetails
//////////////////////////////////////////////////////////////////////
struct join_sources_wf
{
  typedef std::vector<size_t> result_type;
  template<typename Reference1, typename Reference2>
  result_type operator()(Reference1 x, Reference2 y) const
  {
    std::vector<size_t> value, xv(x), yv(y);
    for (auto const& e : xv)
      value.push_back(e);
    for (auto const& e : yv)
      value.push_back(e);
    return value;
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
class update_func
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
  update_func(UpdateConditionWf const& update_condition = UpdateConditionWf(),
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
                           target.property())) {
      target.property().incoming_sweeps_add(m_traversal_id, m_direction,
                                            m_value);
      // A vertex shouldn't be processed until the values for all incoming edges
      // for a given sweep have arrived.
      if (target.property().get_num_incoming_edges_for_direction(m_direction)
          == target.property().incoming_sweeps_size(m_traversal_id,
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
          class DirectionType, class ValueType>
struct sweep_wf
{
  sweep_wf(UpdatePropWf const& update_prop_wf,
           UpdateCondWf const& update_cond_wf,
           FilterEdgeWf const& filter_edge_wf)
    : m_update_prop_wf(update_prop_wf),
      m_update_cond_wf(update_cond_wf),
      m_filter_edge_wf(filter_edge_wf)
  {}

  typedef bool result_type;

  template<typename Vertex, typename GraphVisitor>
  bool operator()(Vertex&& v, GraphVisitor&& graph_visitor) const
  {
    size_t traversal_id;
    bool was_active = false;

    // For each incoming sweep, compute the new value and propagate.
    for (auto const& element : v.property().incoming_sweeps()) {
      was_active = true;
      traversal_id = element.first;
      for (auto const& directional_element : element.second) {
        auto direction = directional_element.first;
        // Compute and update the vertex's property for this sweep.
        // Also return the value to propagate for this sweep.
        auto new_sweep_value =
          m_update_prop_wf(traversal_id, direction, directional_element.second,
                           v.property());
        update_func<typename Vertex::vertex_descriptor,
                    DirectionType, ValueType,
                    UpdateCondWf>
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
    v.property().incoming_sweeps_reset();

    return was_active;
  }

  void define_type(typer& t)
  {
    t.member(m_update_prop_wf);
    t.member(m_update_cond_wf);
    t.member(m_filter_edge_wf);
  }

  UpdatePropWf m_update_prop_wf;
  UpdateCondWf m_update_cond_wf;
  FilterEdgeWf m_filter_edge_wf;
};

} //namespace sweep_algo_detail


//////////////////////////////////////////////////////////////////////
/// @brief A Parallel Multi-Source Sweep algorithm.
///
/// Performs multi-source sweeps on the input @ref graph_view, starting from
/// the given sources. The sweep from each source happens simultaneously, and is
/// identified by a unqiue sweep-Id, which is the descriptor of the source
/// vertex of that sweep. The vertex-properties are initialized by the given
/// @ref init_prop_wf. The sweep visits each vertex, and computes its new
/// property based on the provided @ref update_prop_wf. The sweep then visits
/// the vertex's neighbors that pass the user-provided edge-flitering based on
/// the @ref filter_edge_wf. If the target neighbor vertex should be updated
/// based on the provided @ref update_cond_wf, it continues the sweep, otherwise
/// the neighbor is not updated.
/// @param g The @ref graph_view over the input graph.
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
size_t sweep(GView& g,
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

  // Initialize the vertices and sources. Sources are direction-dependent --
  // a vertex is a source for directions where it has no incoming edges.
  auto new_sources = map_reduce(
    sweep_init_wf<vd_type, DirectionType, ValueType, InitPropWf>(
      {}, directions, init_prop_wf, true), join_sources_wf(), g);
  stapl::do_once([&new_sources]{
      std::cout << " >> num_sources: " << new_sources.size() << std::endl; });
  map_func(
    sweep_init_wf<vd_type, DirectionType, ValueType, InitPropWf>(
        new_sources, directions, init_prop_wf, false), g);


  typedef update_func<vd_type, DirectionType, ValueType, UpdateCondWf>
    update_func_t;
  return graph_paradigm(
      sweep_wf<UpdatePropWf, UpdateCondWf, FilterEdgeWf,
               DirectionType, ValueType>(
          update_prop_wf, update_cond_wf, filter_edge_wf),
      update_func_t(update_cond_wf), g, k);
}

} //namespace stapl

#endif

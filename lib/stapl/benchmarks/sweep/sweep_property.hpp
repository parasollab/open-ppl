/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_CONTAINERS_GRAPH_ALGORITHMS_SWEEP_PROPERTY_HPP
#define STAPL_CONTAINERS_GRAPH_ALGORITHMS_SWEEP_PROPERTY_HPP

#include <stapl/views/proxy.h>
#include <map>
#include <unordered_map>
#include <vector>

namespace stapl {

namespace properties {

//////////////////////////////////////////////////////////////////////
/// @brief Vertex property for use with @ref sweep().
/// All properties to the Sweep algorithm must provide this API.
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
template<typename Direction>
class sweep_property
{
public:
  typedef size_t sweep_id_t;
  typedef Direction direction_t;
  typedef double value_t;
  // Stores for each sweep, all values from each direction.
  typedef std::map<sweep_id_t,
                   std::map<direction_t,
                            std::vector<value_t>>>
    incoming_sweeps_t;
  // Stores for each sweep, the result value from each direction.
  typedef std::map<sweep_id_t,
                   std::map<direction_t, value_t>>
    sweep_values_t;
  // Stores number of incoming edges per direction.
  typedef std::map<direction_t, size_t> incoming_edges_map_t;

  // Stores incoming edges-coordinates from neighbors.
  typedef std::unordered_map<size_t, direction_t> incoming_coordinates_t;

private:
  incoming_sweeps_t m_incoming_sweeps;
  sweep_values_t m_sweep_values;
  incoming_edges_map_t m_num_incoming_edges;
  direction_t m_coordinates;
  incoming_coordinates_t m_incoming_coordinates;

public:
  inline void set_coordinates(direction_t const& coordinates)
  { m_coordinates = coordinates; }

  inline direction_t get_coordinates() const
  { return m_coordinates; }

  inline void add_incoming_coordinates(size_t const& descriptor,
                                       direction_t const& source_coordinates)
  { m_incoming_coordinates[descriptor] = source_coordinates; }

  inline incoming_coordinates_t get_incoming_coordinates() const
  { return m_incoming_coordinates; }

  inline void clear_incoming_coordinates()
  { m_incoming_coordinates.clear(); }

  inline void set_value(sweep_id_t const& sweep_id,
                        direction_t const& direction,
                        value_t const& value)
  { m_sweep_values[sweep_id][direction] = value; }

  inline value_t get_value(sweep_id_t const& sweep_id,
                           direction_t const& direction) const
  {
    auto it = m_sweep_values.find(sweep_id);
    if (it != m_sweep_values.end()) {
      auto it2 = it->second.find(direction);
      if (it2 != it->second.end()) {
        return it2->second;
      }
    }
    std::stringstream ss;
    ss << "get_value: Unknown sweep-id: Sweep-id " << sweep_id
       << " not found in map\n";
    abort(ss.str());
    return value_t();
  }

  inline void incoming_sweeps_add(sweep_id_t const& sweep_id,
                                  direction_t const& direction,
                                  value_t const& value)
  { m_incoming_sweeps[sweep_id][direction].push_back(value); }

  inline incoming_sweeps_t incoming_sweeps() const
  { return m_incoming_sweeps; }

  inline size_t incoming_sweeps_size(sweep_id_t const& sweep_id,
                                     direction_t const& d) const
  {
    auto it = m_incoming_sweeps.find(sweep_id);
    if (it != m_incoming_sweeps.end()) {
      auto it2 = it->second.find(d);
      if (it2 != it->second.end()) {
        return it2->second.size();
      }
    }
    abort("incoming_sweeps_size: Unknown sweep-id: Id not found in map\n");
    return 0;
  }

  inline sweep_values_t sweep_values() const
  { return m_sweep_values; }

  inline void incoming_sweeps_reset()
  { m_incoming_sweeps.clear(); }

  inline size_t get_num_incoming_edges_for_direction(direction_t const& d) const
  {
    auto it = m_num_incoming_edges.find(d);
    if (it != m_num_incoming_edges.end())
      return it->second;
    std::stringstream ss;
    ss << "Unknown direction: (";
    for (auto const& e : d)
      ss << e << ".";
    ss << "). Direction not found in map\n";
    abort(ss.str());
    return 0;
  }

  inline void incr_num_incoming_edges_for_direction(direction_t const& d)
  { m_num_incoming_edges[d]++; }

  inline void set_num_incoming_edges_for_direction(direction_t const& d,
                                                   size_t n)
  { m_num_incoming_edges[d] = n; }

  void define_type(stapl::typer& t)
  {
    t.member(m_incoming_sweeps);
    t.member(m_sweep_values);
    t.member(m_num_incoming_edges);
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Edge property for use with @ref sweep().
/// All properties to the Sweep algorithm must provide this API.
/// @ingroup pgraphAlgoProps
//////////////////////////////////////////////////////////////////////
template<typename Direction>
class sweep_edge_property
{
public:
  typedef Direction direction_t;

private:
  direction_t m_direction;
  bool m_cross_edge = false;

public:
  sweep_edge_property(const direction_t& direction = direction_t())
    : m_direction(direction)
  {}

  inline void set_direction(direction_t const& direction)
  { m_direction = direction; }

  inline direction_t get_direction() const
  { return m_direction; }

  inline bool get_cross_edge() const
  {
    return m_cross_edge;
  }

  inline void set_cross_edge(bool b)
  { m_cross_edge = b; }

  void define_type(stapl::typer& t)
  {
    t.member(m_direction);
    t.member(m_cross_edge);
  }
};

} //namespace properties


// Proxies.
template <class Accessor, class Direction>
class proxy<properties::sweep_property<Direction>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::sweep_property<Direction> target_t;

public:
  typedef size_t sweep_id_t;
  typedef typename target_t::direction_t direction_t;
  typedef typename target_t::value_t value_t;
  typedef typename target_t::incoming_coordinates_t incoming_coordinates_t;
  typedef typename target_t::incoming_sweeps_t incoming_sweeps_t;
  typedef typename target_t::sweep_values_t sweep_values_t;

  inline explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  inline operator target_t(void) const
  { return Accessor::read(); }

  inline proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  inline proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  inline direction_t get_coordinates() const
  { return Accessor::const_invoke(&target_t::get_coordinates); }

  inline void set_coordinates(direction_t const& direction)
  { Accessor::invoke(&target_t::set_coordinates, direction); }

  inline void add_incoming_coordinates(size_t const& descriptor,
                                       direction_t const& source_coordinates)
  {
    Accessor::invoke(&target_t::add_incoming_coordinates, descriptor,
                     source_coordinates);
  }

  inline incoming_coordinates_t get_incoming_coordinates() const
  { return Accessor::const_invoke(&target_t::get_incoming_coordinates); }

  inline void clear_incoming_coordinates()
  { Accessor::invoke(&target_t::clear_incoming_coordinates); }

  inline value_t get_value(sweep_id_t const& sweep_id,
                           direction_t const& direction) const
  { return Accessor::const_invoke(&target_t::get_value, sweep_id, direction); }

  inline void set_value(sweep_id_t const& sweep_id,
                        direction_t const& direction,
                        value_t const& value)
  { Accessor::invoke(&target_t::set_value, sweep_id, direction, value); }

  inline void incoming_sweeps_add(sweep_id_t const& sweep_id,
                                  direction_t const& direction,
                                  value_t const& value)
  {
    Accessor::invoke(&target_t::incoming_sweeps_add, sweep_id, direction,
                     value);
  }

  inline incoming_sweeps_t incoming_sweeps() const
  { return Accessor::const_invoke(&target_t::incoming_sweeps); }

  inline size_t incoming_sweeps_size(sweep_id_t const& sid,
                                     direction_t const& d) const
  { return Accessor::const_invoke(&target_t::incoming_sweeps_size, sid, d); }

  inline sweep_values_t sweep_values() const
  { return Accessor::const_invoke(&target_t::sweep_values); }

  inline void incoming_sweeps_reset()
  { Accessor::invoke(&target_t::incoming_sweeps_reset); }

  inline size_t get_num_incoming_edges_for_direction(direction_t const& d) const
  {
    return Accessor::const_invoke(
        &target_t::get_num_incoming_edges_for_direction, d);
  }

  inline void incr_num_incoming_edges_for_direction(direction_t const& d)
  { Accessor::invoke(&target_t::incr_num_incoming_edges_for_direction, d); }

  inline void set_num_incoming_edges_for_direction(direction_t const& d,
                                                   size_t n)
  { Accessor::invoke(&target_t::set_num_incoming_edges_for_direction, d, n); }

}; //class sweep_property proxy

template <class Accessor, class Direction>
class proxy<properties::sweep_edge_property<Direction>, Accessor>
  : public Accessor
{
private:
  friend class proxy_core_access;
  typedef properties::sweep_edge_property<Direction> target_t;

public:
  typedef typename target_t::direction_t direction_t;

  inline explicit proxy(Accessor const& acc)
    : Accessor(acc)
  { }

  inline operator target_t(void) const
  { return Accessor::read(); }

  inline proxy const& operator=(proxy const& rhs)
  { Accessor::write(rhs); return *this; }

  inline proxy const& operator=(target_t const& rhs)
  { Accessor::write(rhs); return *this;}

  inline direction_t get_direction() const
  { return Accessor::const_invoke(&target_t::get_direction); }

  inline void set_direction(direction_t const& direction)
  { Accessor::invoke(&target_t::set_direction, direction); }

  inline bool get_cross_edge() const
  {
    return Accessor::const_invoke(&target_t::get_cross_edge);
  }

  inline void set_cross_edge(bool b)
  { Accessor::invoke(&target_t::set_cross_edge, b); }

}; //class sweep_property proxy

} //namespace stapl


#endif

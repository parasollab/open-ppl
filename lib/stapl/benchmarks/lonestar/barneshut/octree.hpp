/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_BH_OCTREE
#define STAPL_BENCHMARK_LONESTAR_BH_OCTREE

#include <stapl/runtime/runtime.hpp>
#include <stapl/views/proxy_macros.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>

#include "point.hpp"
#include "utility.hpp"

//////////////////////////////////////////////////////////////////////
/// @brief A node of the octree which contains the center of mass, total
///        mass and the index of the corresponding particle if the node
///        is a leaf.
//////////////////////////////////////////////////////////////////////
class octree_node
{
  /// The center of mass of the cell
  point m_coord;
  /// The total mass of the cell
  double m_mass;
  /// The index of the particle if the node is a leaf. If the node is internal,
  /// then this will be a sentinel value.
  std::size_t m_particle_gid;
  /// Flag used during traversals of the octree to determine if a node is
  /// actively participating in a traversal
  bool m_active;

  static std::size_t invalid_particle()
  {
    return std::numeric_limits<std::size_t>::max();
  }

public:
  //////////////////////////////////////////////////////////////////////
  /// @brief Construct an octree node.
  ///
  /// @param coord The initial center of mass of this node.
  /// @param mass The mass of the cell.
  /// @param particle_gid The index of the particle this node is associated
  ///        with.
  //////////////////////////////////////////////////////////////////////
  octree_node(point const& coord=point(), double mass = 0.,
              std::size_t particle_gid = invalid_particle())
   : m_coord(coord), m_mass(mass), m_particle_gid(particle_gid),
     m_active(false)
  { }

  //////////////////////////////////////////////////////////////////////
  /// @brief Add the contribution of another particle or node to
  ///        this node by updating the center of mass and total mass.
  ///
  /// @param coord Coordinate of the other node
  /// @param mass Mass of the other node
  //////////////////////////////////////////////////////////////////////
  void update_center_of_mass(point const& coord, double mass)
  {
    if (mass > 0)
      m_coord = center_of_mass(m_coord, coord, m_mass, mass);
    else
      m_coord = coord;

    m_mass += mass;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Convert this node to an internal node
  //////////////////////////////////////////////////////////////////////
  void internalize(void)
  {
    m_particle_gid = invalid_particle();
    m_mass = 0;
    m_coord = point(0., 0., 0.);
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set this node as actively being traversed
  //////////////////////////////////////////////////////////////////////
  void activate(void)
  {
    m_active = true;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Set this node as finished with the traversal
  //////////////////////////////////////////////////////////////////////
  void deactivate(void)
  {
    m_active = false;
  }

  bool is_leaf(void) const
  {
    return m_particle_gid != invalid_particle();
  }

  bool is_active(void) const
  {
    return m_active;
  }

  point coord(void) const
  {
    return m_coord;
  }

  void coord(point const& c)
  {
    m_coord = c;
  }

  void mass(double m)
  {
    m_mass = m;
  }

  double mass(void) const
  {
    return m_mass;
  }

  std::size_t particle_id(void) const
  {
    return m_particle_gid;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_coord);
    t.member(m_mass);
    t.member(m_particle_gid);
    t.member(m_active);
  }
};


namespace stapl {

STAPL_PROXY_HEADER(octree_node)
{
  STAPL_PROXY_DEFINES(octree_node)
  STAPL_PROXY_METHOD_RETURN(mass, double)
  STAPL_PROXY_METHOD_RETURN(coord, point)
  STAPL_PROXY_METHOD_RETURN(is_leaf, bool)
  STAPL_PROXY_METHOD_RETURN(particle_id, std::size_t)
  STAPL_PROXY_METHOD(mass, double)
  STAPL_PROXY_METHOD(coord, point)
  STAPL_PROXY_METHOD(update_center_of_mass, point, double)
  STAPL_PROXY_METHOD(internalize)
  STAPL_PROXY_METHOD_RETURN(is_active, bool)
  STAPL_PROXY_METHOD(activate)
  STAPL_PROXY_METHOD(deactivate)
};

} //namespace stapl


typedef stapl::dynamic_graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
          octree_node, std::pair<bool, int>
        > octree_graph;

typedef stapl::graph_view<octree_graph> octree_view;

#endif

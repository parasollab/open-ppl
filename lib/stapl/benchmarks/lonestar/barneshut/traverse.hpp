/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_BH_TREE_TRAVERSE
#define STAPL_BENCHMARK_LONESTAR_BH_TREE_TRAVERSE

#include <stapl/algorithms/algorithm.hpp>
#include <stapl/views/repeated_view.hpp>

//////////////////////////////////////////////////////////////////////
///  @file traverse.hpp
///  @brief Implementation of traversing an octree to compute forces of
///         particles.
///
/// First, every particle initiates a traversal from the root of the tree.
/// During the traversal, the particle examines the node and determines what
/// to do in the following manner:
///
///  - If the node is a leaf, or it is far enough away to use it as an
///    approximation, a task is spawned on the particle to add the
///    contribution of the node.
///  - Otherwise, tasks are created over all of the node's children
///    with the same workfunction
//////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////
/// @brief Work function that is applied on a particle after a node
///        in the octree is found to be far enough to approximate (or
///        it's a leaf).
///
///        The actual computation that is applied to the particle is the
///        standard Newtonian gravitational equation.
//////////////////////////////////////////////////////////////////////
class compute_wf
{
  point m_coord;
  double m_mass;

public:
  compute_wf(point c, double m)
    : m_coord(c), m_mass(m)
  { }

  typedef void result_type;

  template<typename P>
  void operator()(P p)
  {
    point coord = p.coord();

    double dx = coord.x - m_coord.x;
    double dy = coord.y - m_coord.y;
    double dz = coord.z - m_coord.z;
    double d2 = dx*dx + dy*dy + dz*dz;

    stapl_assert(d2 > 0, "two bodies are the same");

    double id = 1 / std::sqrt(d2);
    double id3 =  id*id*id;

    double f = m_mass * id3;

    double fx = dx * f;
    double fy = dy * f;
    double fz = dz * f;

    point acc = p.acceleration();
    acc.plus(point(fx, fy, fz));
    p.acceleration(acc);
  }

  void define_type(stapl::typer &t)
  {
    t.member(m_coord);
    t.member(m_mass);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Primary octree traversal work function that recursively
///        adds tasks to traverse children of the node if it cannot
///        be approximated. If a node can be approximated (or it's
///        a leaf), we spawn a task to add the contribution to the
///        particle that is traversing.
//////////////////////////////////////////////////////////////////////
class traverse_wf
  : public stapl::dynamic_wf
{
  std::size_t m_pid;
  std::size_t m_level;
  point m_coord;

public:
  typedef void result_type;

  traverse_wf(std::size_t id, std::size_t level, point coord)
    : m_pid(id), m_level(level), m_coord(coord)
  {}

  void define_type(stapl::typer &t)
  {
    t.member(m_pid);
    t.member(m_level);
    t.member(m_coord);
  }

  template<typename TGV, typename CellNode, typename CellGraphV,
           typename ParticleView>
  void operator()(TGV const& tgv, CellNode cell_node, CellGraphV& cellgraph_view,
                  ParticleView& particle_view)
  {
    // if this octree node is a leaf, or is far enough away to approximated
    if (cell_node.property().is_leaf() ||
        can_approx(cell_node.property().coord(), m_coord, m_level))
    {
      // compute the forces of this particle (if it's not the same particle
      // that is traversing the tree)
      if (cell_node.property().particle_id() != m_pid)
        // add a task to add the contribution of this octree node
        // to the particle in the view
        tgv.add_task(
          compute_wf(
            cell_node.property().coord(),
            cell_node.property().mass()
          ),
          localize_ref(particle_view, m_pid)
        );
    }
    // this is an internal node
    else
    {
      // create traversal tasks over this node's children
      for (auto&& child : cell_node)
        if (child.property().first)
          tgv.add_task(traverse_wf(m_pid, m_level+1, m_coord),
            localize_ref(tgv),
            localize_ref(cellgraph_view, child.target()),
            localize_ref(cellgraph_view),
            localize_ref(particle_view)
          );
    }
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function that initiates the traversal from the root node
///        for every particle.
//////////////////////////////////////////////////////////////////////
class traverse_octree
  : public stapl::dynamic_wf
{
  std::size_t m_base_node;

public:
  typedef void result_type;

  traverse_octree(std::size_t base_node)
   : m_base_node(base_node)
  { }

  template<typename TGV, typename P, typename CellGraphV, typename ParticleView>
  void operator()(TGV const& tgv, P particle, CellGraphV& cellgraph_view,
                  ParticleView& particle_view)
  {
    // create a task for this particle to traverse the octree from the root
    tgv.add_task(
      traverse_wf(index_of(particle), 0, particle.coord()),
      localize_ref(tgv),
      localize_ref(cellgraph_view, m_base_node),
      localize_ref(cellgraph_view),
      localize_ref(particle_view)
    );
  }

  void define_type(stapl::typer& t)
  {
    t.base<stapl::dynamic_wf>(*this);
    t.member(m_base_node);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function that moves the position of a particle based
///        on the force that was computed in the previous phase.
//////////////////////////////////////////////////////////////////////
struct update_positions
{
  typedef void result_type;

  double m_dt;

  update_positions(double const& dt = 0.1)
   : m_dt(dt)
  { }

  template<typename T>
  void operator()(T x)
  {
    point f = x.acceleration();
    point v = x.velocity();
    point p = x.coord();

    double ax = f.x / x.mass();
    double ay = f.y / x.mass();
    double az = f.z / x.mass();

    double vx = v.x + m_dt * ax;
    double vy = v.y + m_dt * ay;
    double vz = v.z + m_dt * az;

    double px = p.x + m_dt * vx;
    double py = p.y + m_dt * vy;
    double pz = p.z + m_dt * vz;

    x.coord(point(px, py, pz));
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_dt);
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Work function that sets a particle's force to zero
//////////////////////////////////////////////////////////////////////
struct reset_force
{
  typedef void result_type;

  template<typename T>
  void operator()(T x)
  {
    x.acceleration(point(0., 0., 0.));
  }
};


//////////////////////////////////////////////////////////////////////
/// @brief Given an octree and a view of particles, compute the interactions
///        between the particles, approximating when appropriate using
///        the octree.
///
/// @param oct The octree view
/// @param root_id The descriptor of the root in the octree
/// @param particle_view The view of the particles
/// @param dt Delta t parameter that controls how much time to simulate
//////////////////////////////////////////////////////////////////////
template<typename Octree, typename ParticleView>
void compute_forces(Octree& oct, std::size_t root_id,
                    ParticleView particle_view, double dt)
{
  // traverse the octree starting from the root
  map_func(traverse_octree(root_id), particle_view, make_repeat_view(oct),
    make_repeat_view(particle_view)
  );

  // update the positions of the particles based on the
  // calculated forces of the particles
  for_each(particle_view, update_positions(dt));

  // reset the forces of the particles
  for_each(particle_view, reset_force());
}

#endif

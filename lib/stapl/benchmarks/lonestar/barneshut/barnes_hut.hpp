/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARK_LONESTAR_BH
#define STAPL_BENCHMARK_LONESTAR_BH

#include <stapl/containers/graph/algorithms/rebalance_global.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "particle.hpp"
#include "octree.hpp"
#include "display.hpp"

#include "center_of_mass.hpp"
#include "build.hpp"
#include "traverse.hpp"

//////////////////////////////////////////////////////////////////////
/// @brief Perform an n-body simulation on a group of particles using
///        the Barnes-Hut tree-based method.
///
/// @param particle_view View of the particles
/// @param steps The number of time steps for which the simulation should
///              be carried out.
/// @param dt The delta t parameter that controls how much time is
///           simulated per time step.
//////////////////////////////////////////////////////////////////////
template<typename ParticleView>
void barnes_hut(ParticleView particle_view, std::size_t steps, double dt)
{
  using namespace stapl;

  // create octree and view
  octree_graph g;
  octree_view oct(g);

  // time step loop
  for (std::size_t i = 0; i < steps; ++i)
  {
    // build the octree from the particle view
    std::size_t root_id = build_octree(oct, particle_view);

    // build_octree invalidates oct, we need to reconstruct it
    oct = octree_view(g);

    // rebalance the octree
    rebalance_global(oct);

    // create a new view because rebalance modified the version
    octree_view balanced_oct(g);

    // compute center of mass information for
    // internal nodes in the octree
    update_centers_of_mass(balanced_oct);

    // compute force information by traversing the tree
    // and updating the particles with the forces
    compute_forces(balanced_oct, root_id, particle_view, dt);
  }
}

#endif

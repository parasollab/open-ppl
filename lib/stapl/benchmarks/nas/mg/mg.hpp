/*
// Copyright (c) 2000-2010, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_NAS_MG
#define STAPL_BENCHMARKS_NAS_MG

#include <stapl/utility/tuple.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "random.hpp"
#include "utils.hpp"
#include "stencils.hpp"

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief V-cycle computation for the multigrid computation.
///
/// @param u_grids Container of solution grids
/// @param r_grids Container of residual grids
/// @param v       Single input grid
//////////////////////////////////////////////////////////////////////
template<typename UGrids, typename RGrids, typename VGrid>
void v_cycle(UGrids& u_grids, RGrids& r_grids, VGrid& v)
{
  // (1) down cycle (restrict the residual from the fine grid to the coarse).
  for (size_t idx=0; idx < r_grids.size()-1; ++idx)
  {
    rprj3(r_grids[idx], r_grids[idx+1]);
  }


  // (2) compute an approximate solution on the coarsest grid
  zero3(u_grids.back());
  psinv(r_grids.back(), u_grids.back());

  // (3) upcycle
  for (int idx=u_grids.size()-1; idx>1; --idx)
  {
    // (3a) prolongate from level k-1 to k
    zero3(u_grids[idx-1]);
    interp(u_grids[idx], u_grids[idx-1]);

    // (3b) compute residual for level k
    resid(u_grids[idx-1], r_grids[idx-1], r_grids[idx-1]);

    // (3c) apply smoother
    psinv(r_grids[idx-1], u_grids[idx-1]);
  }

  // final level of upcycle
  interp(u_grids[1], u_grids[0]);
  resid(u_grids[0], v, r_grids[0]);
  psinv(r_grids[0], u_grids[0]);
}


//////////////////////////////////////////////////////////////////////
/// @brief Compute the approximate solution to a scalar Poisson problem
///        on a discrete 3D grid with periodic boundary conditions using
///        the multigrid method.
///
/// @param v Input grid.
/// @param iters Number of v-cycle iterations.
/// @return A tuple of the solution grid and the residual grid
//////////////////////////////////////////////////////////////////////
template<typename Grid>
std::tuple<Grid, Grid> mg(Grid const& v, const std::size_t iters)
{
  typedef std::vector<Grid> multi_grid_views_type;

  // log(n) grids for the solution and the residual
  multi_grid_views_type u_grids;
  multi_grid_views_type r_grids;
  setup(u_grids, r_grids, get<0>(v.dimensions()));

  // compute random values for the right hand side
  zran3(v);

  // zero out the finest solution
  zero3(u_grids[0]);

  stapl::rmi_fence();

  stapl::counter<stapl::default_timer> tmr;
  tmr.start();

  // compute fine grained residual
  resid(u_grids[0], v, r_grids[0]);

  // perform multiple iterations of v-cycle
  for (std::size_t iterations = 0; iterations < iters; ++iterations)
  {
    if (stapl::get_location_id() == 0)
      std::cout << "Iteration: " << iterations + 1 << "\n";

    // main v-cycle computation
    v_cycle(u_grids, r_grids, v);
    resid(u_grids[0], v, r_grids[0]);
  }

  double t = tmr.stop();

  stapl::do_once([&](){
      std::cout << "Total time: " << t << std::endl;;
    });

  return std::make_tuple(u_grids[0], r_grids[0]);
}

#endif

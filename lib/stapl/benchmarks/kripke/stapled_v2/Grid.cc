/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "Grid.h"
#include "Kripke/Input_Variables.h"

Grid_Data_Base::Grid_Data_Base(Input_Variables *input_vars,
                               Directions *directions)
  : m_px(input_vars->npx), m_py(input_vars->npy), m_pz(input_vars->npz),
    m_ax(input_vars->ax), m_ay(input_vars->ay), m_az(input_vars->az),
    m_cx(input_vars->nx/m_ax), m_cy(input_vars->ny/m_ay),
    m_cz(input_vars->nz/m_az), m_num_moments(input_vars->legendre_order),
    m_nesting(input_vars->nesting)
{
  // My processor id and ijk
  int loc_id = this->get_location_id();

  // delinearize the location id and initialize m_mynbr.
  // We're using a zyx-major numbering of locations, similar to PDT.
  int ksub_ref = loc_id % m_pz;
  int jsub_ref = ((loc_id - ksub_ref) / m_pz) % m_py;
  int isub_ref = (loc_id - ksub_ref - m_pz*jsub_ref) / (m_pz * m_py);

  if(ksub_ref == 0)
    m_mynbr[2][0] = -1;
  else
    m_mynbr[2][0] = loc_id - 1;

  if(ksub_ref == m_pz-1)
    m_mynbr[2][1] = -1;
  else
    m_mynbr[2][1] = loc_id + 1;

  if(jsub_ref == 0)
    m_mynbr[1][0] = -1;
  else
    m_mynbr[1][0] = loc_id - m_pz;

  if(jsub_ref == m_py-1)
    m_mynbr[1][1] = -1;
  else
    m_mynbr[1][1] = loc_id + m_pz;

  if(isub_ref == 0)
    m_mynbr[0][0] = -1;
  else
    m_mynbr[0][0] = loc_id - m_pz * m_py;

  if(isub_ref == m_px-1)
    m_mynbr[0][1] = -1;
  else
    m_mynbr[0][1] = loc_id + m_pz * m_py;

  computeGrid(0, m_cx < m_px ? m_cx : m_px, input_vars->nx, isub_ref, 0.0, 1.0);
  computeGrid(1, m_cy < m_py ? m_cy : m_py, input_vars->ny, jsub_ref, 0.0, 1.0);
  computeGrid(2, m_cz < m_pz ? m_cz : m_pz, input_vars->nz, ksub_ref, 0.0, 1.0);

  m_num_zones = m_nzones[0] * m_nzones[1] * m_nzones[2];

  computeSweepIndexSets();
} // Grid_Data_Base::Grid_Data_Base


void Grid_Data_Base::computeGrid(int dim, int num_locs, int num_zones,
       stapl::location_type loc_index, double min_coord, double max_coord)
{
  /* Calculate unit roundoff and load into grid_data */
  double eps = 1e-32;
  double thsnd_eps = 1000.e0*(eps);

  // Compute subset of global zone indices
  unsigned int num_local_zones = num_zones / num_locs;
  unsigned int rem = num_zones % num_locs;
  unsigned int ilower, iupper;
  if (rem != 0) {
    if (loc_index < rem){
      ++num_local_zones;
      ilower = loc_index * num_local_zones;
    }
    else
      ilower = rem + loc_index * num_local_zones;
  }
  else
  {
    ilower = loc_index * num_local_zones;
  }

  iupper = ilower + num_local_zones - 1;

  // allocate grid deltas
  m_deltas[dim].resize(num_local_zones+2);

  // Compute the spatial grid
  double dx = (max_coord - min_coord) / num_zones;
  double coord_lo = min_coord + (ilower) * dx;
  double coord_hi = min_coord + (iupper+1) * dx;
  for(unsigned int i = 0; i < num_local_zones+2; i++){
    m_deltas[dim][i] = dx;
  }
  if(std::abs(coord_lo - min_coord) <= thsnd_eps*std::abs(min_coord)){
    m_deltas[dim][0] = 0.0;
  }
  if(std::abs(coord_hi - max_coord) <= thsnd_eps*std::abs(max_coord)){
    m_deltas[dim][num_local_zones+1] = 0.0;
  }

  m_nzones[dim] = num_local_zones;
}


/**
 * Computes index sets for each octant, and each tile (experimental).
 * Determines logical indices, and increments for i,j,k based on grid
 * information and quadrature set sweeping direction.
 */
void Grid_Data_Base::computeSweepIndexSets(void){
  m_octant_extent.resize(8);
  for(int octant = 0;octant < 8;++ octant){

    int id, jd, kd;
    switch(octant){
      case 0: id = 1; jd = 1; kd = 1; break;
      case 1: id = -1; jd = 1; kd = 1; break;
      case 2: id = -1; jd = -1; kd = 1; break;
      case 3: id = 1; jd = -1; kd = 1; break;
      case 4: id = 1; jd = 1; kd = -1; break;
      case 5: id = -1; jd = 1; kd = -1; break;
      case 6: id = -1; jd = -1; kd = -1; break;
      case 7: id = 1; jd = -1; kd = -1; break;
    }

    int istartz, istopz, in;

    if(id > 0){
      istartz = 0; istopz = m_nzones[0]-1; in = 1;
    }
    else {
      istartz = m_nzones[0]-1; istopz = 0; in = -1;
    }

    int jstartz, jstopz, jn;
    if(jd > 0){
      jstartz = 0; jstopz = m_nzones[1]-1; jn = 1;
    }
    else {
      jstartz = m_nzones[1]-1; jstopz = 0; jn = -1;
    }

    int kstartz, kstopz, kn;
    if(kd > 0){
      kstartz = 0; kstopz = m_nzones[2]-1; kn =  1;
    }
    else {
      kstartz = m_nzones[2]-1; kstopz = 0; kn = -1;
    }

    // Define extent block and pattern
    Grid_Sweep_Block &extent = m_octant_extent[octant];
    extent.start_i = istartz;
    extent.start_j = jstartz;
    extent.start_k = kstartz;
    extent.end_i = istopz + in;
    extent.end_j = jstopz + jn;
    extent.end_k = kstopz + kn;
    extent.inc_i = in;
    extent.inc_j = jn;
    extent.inc_k = kn;
  }
}

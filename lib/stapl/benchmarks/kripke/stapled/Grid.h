/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_KRIPKE_GRID3D_H
#define STAPL_BENCHMARKS_KRIPKE_GRID3D_H

// System includes
#include <iostream>
#include <vector>

#include "BaseGrid.h"
#include "Cell.hpp"
#include "Kripke/Input_Variables.h"
#include "Kripke/Kernel.h"


////////////////////////////////////////////////////////////////////////////////
///
/// @brief Represents a hexahedral grid.
///
////////////////////////////////////////////////////////////////////////////////
class Grid_Data : public BaseGrid
{
private:
  int px, py, pz;

  // Total Number of zones in the grid on this location
  int m_num_zones;

  // Number of zones in each dimension on this location
  int m_nzones[3];

  // Neighboring locations in each dimension
  int m_mynbr[3][2];

  // Spatial grid deltas in each dimension
  // TGS: Does Kripke support variable width zones?
  // TGS: constructed in computeGrid.
  std::vector<double> m_deltas[3];

  // Sweep index sets for each octant
  // TGS : computed in computeSweepIndexSets
  std::vector<Grid_Sweep_Block> m_octant_extent;

  // Group/Angle sets
  std::vector<std::vector<Group_Dir_Set> > m_gd_sets;

  int m_num_moments;

  // n, m indicies for traversing ell, ell_plus of vertices
  std::vector<int> m_nm_table;

  void computeSweepIndexSets(int block_size);
public:
  typedef Cell3D                               cell_type;
  typedef Vector3D                             VectorND;

  typedef BaseGrid                             base_grid_type;
  typedef base_grid_type::lcl_cell_iterator    lcl_cell_iterator;
  typedef base_grid_type::cell_iterator        cell_iterator;
  typedef base_grid_type::lcl_edge_iterator    lcl_edge_iterator;
  typedef base_grid_type::edge_iterator        edge_iterator;
  typedef base_grid_type::cell_iterator        vertex_iterator;


  /*!
    \brief Build a grid from already parsed data.
  */
  Grid_Data(Input_Variables *input_vars, Directions *directions);

  int num_zones(void) const
  { return m_num_zones; }

  int nzones(int dim) const
  { return m_nzones[dim]; }

  int (&nzones(void))[3]
  { return m_nzones; }

  int (&mynbr(void))[3][2]
  { return m_mynbr; }

  std::vector<double> const& deltas(int dim) const
  { return m_deltas[dim]; }

  std::vector<Grid_Sweep_Block> const& octant_extent(void) const
  { return m_octant_extent; }

  std::vector<std::vector<Group_Dir_Set> >& gd_sets(void)
  { return m_gd_sets; }

  std::vector<std::vector<Group_Dir_Set> > const& gd_sets(void) const
  { return m_gd_sets; }

  int num_moments(void) const
  { return m_num_moments; }

  std::vector<int> const& nm_table(void) const
  { return m_nm_table; }

  std::vector<int>& nm_table(void)
  { return m_nm_table; }

  void computeGrid(int dim, int num_locs, int num_zones,
         stapl::location_type loc_index, double min_coord, double max_coord);

  void randomizeData(void)
  {
    // TGS: note, for full validation with Kripke we need to match their random
    // initialization.
  }

  bool compare(Grid_Data const&, double tolerance, bool verbose);

  void copy(Grid_Data const&);
};

#endif

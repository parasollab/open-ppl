/*--------------------------------------------------------------------------
 * Header file for the Input_Variables structure
 *--------------------------------------------------------------------------*/

#ifndef KRIPKE_INPUT_VARIABLES_H__
#define KRIPKE_INPUT_VARIABLES_H__

#include <Kripke.h>
#include <string>

/**
 * This structure defines the input parameters to setup a problem.
 *
 * npx                   :
 * npy                   : The number of processors in the y-direction.
 * npz                   : The number of processors in the z-direction.
 * nx                    : Number of spatial zones in the x-direction.
 * ny                    : Number of spatial zones in the y-direction.
 * nz                    : Number of spatial zones in the z-direction.
 * niter                 : The number of sweep iterations.
 * num_direction_per_octant : The number of directions per octant
 */

struct Input_Variables {
  int npx, npy, npz;            // The number of processors in x,y,z
  int nx, ny, nz;               // Number of spatial zones in x,y,z
  int ax, ay, az;               // Number of spatial zones per zone set in x,y,z
  int num_zonesets_dim[3];      // Number of zoneset in x, y, z

  int num_directions;           // Total number of directions
  int num_dirsets;              // Number of direction sets
  int num_dirs_per_dirset;

  int num_groups;               // Total number of energy groups
  int num_groupsets;
  int num_groups_per_groupset;

  int niter;                    // number of solver iterations to run
  int legendre_order;           // Scattering order (number Legendre coeff's - 1)
  Nesting_Order nesting;

  // new variables
  int quad_num_polar;           // Number of polar quadrature points (0 for dummy)
  int quad_num_azimuthal;       // Number of azimuthal quadrature points (0 for dummy)
  int layout_pattern;           // Which subdomain/task layout to use
  ParallelMethod parallel_method;
  double sigt[3];               // total cross section for 3 materials
  double sigs[3];               // total scattering cross section for 3 materials
  std::string run_name;         // Name to use when generating output files
#ifdef KRIPKE_USE_SILO
  std::string silo_basename;    // name prefix for silo output files
#endif

  Input_Variables();

  bool checkValues(void) const;
};

#endif

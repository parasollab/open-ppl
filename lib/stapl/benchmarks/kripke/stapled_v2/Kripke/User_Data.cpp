/*--------------------------------------------------------------------------
 * Utility functions for the User_Data structure.
 * Note that user input data is only used in this file.
 *--------------------------------------------------------------------------*/

#include <Kripke/User_Data.h>
#include <Kripke/Kernel.h>
#include <Kripke.h>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <stapl/runtime.hpp>

/*--------------------------------------------------------------------------
 * AllocUserData : Creates a new User_Data structure and
 *                       allocates memory for its data.
 *--------------------------------------------------------------------------*/
/*--------------------------------------------------------------------------
 * input_vars      : Structure with input data.
 *--------------------------------------------------------------------------*/

User_Data::User_Data(Input_Variables *input_vars)
{
  /* Set the processor grid dimensions */
  int R = (input_vars->npx) * (input_vars->npy) * (input_vars->npz);
  /* Check size of PQR_group is the same as MPI_COMM_WORLD */
  int size = stapl::get_num_locations();
  if (R != size) {
    int myid = stapl::get_location_id();
    if (myid == 0) {
      printf("ERROR: Incorrect number of MPI tasks. Need %d MPI tasks.", R);
    }
    std::exit(1);
  }

  // Create the spatial grid
  switch (input_vars->nesting) {
    case NEST_DGZ:
      grid_data = new Grid_Data<NEST_DGZ>(input_vars, &directions[0]);
      break;
    case NEST_DZG:
      grid_data = new Grid_Data<NEST_DZG>(input_vars, &directions[0]);
      break;
    case NEST_GDZ:
      grid_data = new Grid_Data<NEST_GDZ>(input_vars, &directions[0]);
      break;
    case NEST_GZD:
      grid_data = new Grid_Data<NEST_GZD>(input_vars, &directions[0]);
      break;
    case NEST_ZDG:
      grid_data = new Grid_Data<NEST_ZDG>(input_vars, &directions[0]);
      break;
    case NEST_ZGD:
      grid_data = new Grid_Data<NEST_ZGD>(input_vars, &directions[0]);
      break;
  }

  // Create base quadrature set
  InitDirections(this, input_vars);
  // printf("Total directions=%d\n", (int)directions.size());

  num_direction_sets = input_vars->num_dirsets;
  num_directions_per_set = input_vars->num_dirs_per_dirset;
  num_group_sets = input_vars->num_groupsets;
  num_groups_per_set = input_vars->num_groups_per_groupset;

  // Initialize Group and Direction Set Structures
  grid_data->gd_sets().resize(input_vars->num_groupsets);
  int group0 = 0;
  for (unsigned int gs = 0; gs < grid_data->gd_sets().size(); ++gs) {
    grid_data->gd_sets()[gs].resize(input_vars->num_dirsets);
    int dir0 = 0;
    for (unsigned int ds = 0; ds < grid_data->gd_sets()[gs].size(); ++ds) {
      Group_Dir_Set &gdset = grid_data->gd_sets()[gs][ds];
      gdset.num_groups = input_vars->num_groups_per_groupset;
      gdset.num_directions = input_vars->num_dirs_per_dirset;

      gdset.group0 = group0;

      gdset.direction0 = dir0;
      gdset.directions = &directions[dir0];

      dir0 += input_vars->num_dirs_per_dirset;
    }

    group0 += input_vars->num_groups_per_groupset;
  }

  /* Set ncalls */
  niter = input_vars->niter;

  // setup cross-sections
  sigma_tot.resize(num_group_sets * num_groups_per_set, 0.0);

  // Compute number of zones
  global_num_zones =
    (size_t)input_vars->nx * (size_t)input_vars->ny * (size_t)input_vars->nz;

  // create the kernel object based on nesting
  kernel = createKernel(input_vars->nesting, 3, grid_data);

  // Allocate data
  switch (input_vars->nesting) {
    case NEST_DGZ:
      kernel->allocateStorage<NEST_DGZ>(this);
      break;
    case NEST_DZG:
      kernel->allocateStorage<NEST_DZG>(this);
      break;
    case NEST_GDZ:
      kernel->allocateStorage<NEST_GDZ>(this);
      break;
    case NEST_GZD:
      kernel->allocateStorage<NEST_GZD>(this);
      break;
    case NEST_ZDG:
      kernel->allocateStorage<NEST_ZDG>(this);
      break;
    case NEST_ZGD:
      kernel->allocateStorage<NEST_ZGD>(this);
      break;
  }

  /* Create buffer info for sweeping if using Diamond-Difference */
  CreateBufferInfo(this);
}

User_Data::~User_Data()
{
  /* Free buffers used in sweeping */
  delete grid_data;
  delete kernel;
}

/*
 * Timing timing;

  int niter;

  double source_value;

  std::vector<double> sigma_tot;            // Cross section data

  Grid_Data_Base *grid_data;                // Spatial grids and variables

  size_t global_num_zones;                  // Total zones across all grids
  int num_group_sets;                       // Number of group-sets
  int num_groups_per_set;                   // How many groups in each set
  int num_direction_sets;                   // Number of direction-sets
  int num_directions_per_set;               // Number of directions per dir set

  std::vector<Directions> directions;       // Direction data
  std::vector<int> octant_map;              // Direction origination octant

  Kernel *kernel;
  Comm *comm;
 */
void User_Data::randomizeData()
{
  for (unsigned int i = 0; i < sigma_tot.size(); ++i) {
    sigma_tot[i] = drand48();
  }

  for (unsigned int i = 0; i < directions.size(); ++i) {
    directions[i].xcos = 1.0 / sqrt(3);
    directions[i].ycos = 1.0 / sqrt(3);
    directions[i].zcos = 1.0 / sqrt(3);
  }

  grid_data->randomizeData();
}
/**
 * Copies DATA without changing nesting
 */
void User_Data::copy(User_Data const &b)
{
  sigma_tot = b.sigma_tot;
  directions = b.directions;
  grid_data->copy(*b.grid_data);
}

bool User_Data::compare(User_Data const &b, double tol, bool verbose)
{
  bool is_diff = false;

  is_diff |= compareVector("sigma_tot", sigma_tot, b.sigma_tot, tol, verbose);

  for (unsigned int i = 0; i < directions.size(); ++i) {
    std::stringstream dirname;
    dirname << "directions[" << i << "]";

    is_diff |= compareScalar(dirname.str() + ".xcos", directions[i].xcos,
                             b.directions[i].xcos, tol, verbose);

    is_diff |= compareScalar(dirname.str() + ".ycos", directions[i].ycos,
                             b.directions[i].ycos, tol, verbose);

    is_diff |= compareScalar(dirname.str() + ".zcos", directions[i].zcos,
                             b.directions[i].zcos, tol, verbose);
  }

  is_diff |= grid_data->compare(*b.grid_data, tol, verbose);

  return is_diff;
}

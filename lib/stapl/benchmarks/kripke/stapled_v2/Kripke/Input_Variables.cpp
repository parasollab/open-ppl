/******************************************************************************
 *
 * Input Variables
 *
 *****************************************************************************/

#include<Kripke/Input_Variables.h>

#include<mpi.h>

/**
* Setup the default input choices
*/
Input_Variables::Input_Variables() :
  npx(1), npy(1), npz(1),
  nx(12), ny(12), nz(12),
  ax(12), ay(12), az(12),

  num_directions(8),
  num_dirsets(8),

  num_groups(4),
  num_groupsets(2),

  niter(1),
  legendre_order(4),

  nesting(NEST_DGZ),

  // new variables
  quad_num_polar(0),
  quad_num_azimuthal(0),
  layout_pattern(0),
  parallel_method(PMETHOD_SWEEP),
  run_name("kripke")
  {
    num_zonesets_dim[0] = 1;
    num_zonesets_dim[1] = 1;
    num_zonesets_dim[2] = 1;

    num_dirs_per_dirset = num_directions / num_dirsets;
    num_groups_per_groupset = num_groups / num_groupsets;

    sigt[0] = 0.1;
    sigt[1] = 0.0001;
    sigt[2] = 0.1;

    sigs[0] = 0.05;
    sigs[1] = 0.00005;
    sigs[2] = 0.05;
  }



/**
 *  Checks validity of inputs, returns 'true' on error.
 */
bool Input_Variables::checkValues(void) const{
  // make sure any output only goes to root
  int rank;
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);

  if(num_zonesets_dim[0] <= 0 || num_zonesets_dim[1] <= 0 || num_zonesets_dim[2] <= 0){
    if(!rank)
      printf("Number of zone-sets in each dim need to be greater than or equal to 1\n");
    return true;
  }

  if(layout_pattern < 0 || layout_pattern > 1){
    if(!rank)
      printf("Layout(%d) must be either 0 or 1\n", layout_pattern);
    return true;
  }

  if(nesting < 0){
    if(!rank)
      printf("Invalid nesting selected\n");
    return true;
  }

  if(num_groups < 1){
    if(!rank)
      printf("Number of groups (%d) needs to be at least 1\n", num_groups);
    return true;
  }

  if(num_groups % num_groupsets){
    if(!rank)
      printf("Number of groups (%d) must be evenly divided by number of groupsets (%d)\n",
        num_groups, num_groupsets);
    return true;
  }

  if(num_directions < 8){
    if(!rank)
      printf("Number of directions (%d) needs to be at least 8\n", num_directions);
    return true;
  }

  if(num_dirsets % 8 && num_dirsets < 8){
    if(!rank)
      printf("Number of direction sets (%d) must be a multiple of 8\n", num_dirsets);
    return true;
  }

  if(num_directions % num_dirsets){
    if(!rank)
      printf("Number of directions (%d) must be evenly divided by number of directionsets(%d)\n",
        num_directions, num_dirsets);
    return true;
  }

  if(legendre_order < 0){
    if(!rank)
      printf("Legendre scattering order (%d) must be >= 0\n", legendre_order);
    return true;
  }

  if(niter < 1){
    if(!rank)
      printf("You must run at least one iteration (%d)\n", niter);
    return true;
  }

  return false;
}



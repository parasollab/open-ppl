/*--------------------------------------------------------------------------
 * Sweep-based solver routine.
 *--------------------------------------------------------------------------*/

#include <Kripke.h>
#include <Kripke/User_Data.h>
#include <Kripke/Kernel/Kernel_3d_ZDG.h>
#include <Kripke/Kernel/Kernel_3d_DGZ.h>
#include <Kripke/Kernel/Kernel_3d_GDZ.h>
#include "explicit_sweep_factory.hpp"
#include <vector>
#include <stdio.h>



/*----------------------------------------------------------------------
 * SweepSolverSolve
 *----------------------------------------------------------------------*/

int SweepSolver (User_Data *user_data)
{

  // TGS : Calls to kernel methods need to be replaced by PARAGRAPHs.

  Kernel *kernel = user_data->kernel;
  Grid_Data *grid_data = user_data->grid_data;

  BLOCK_TIMER(user_data->timing, Solve);

  // Loop over iterations
  for(int iter = 0;iter < user_data->niter;++ iter){

    /*
     * Compute the RHS
     */

    // Discrete to Moments transformation
    {
      BLOCK_TIMER(user_data->timing, LTimes);
      kernel->LTimes(grid_data);
    }

    // This is where the Scattering kernel would go!



    // Moments to Discrete transformation
    {
      BLOCK_TIMER(user_data->timing, LPlusTimes);
      kernel->LPlusTimes(grid_data);
    }

    /*
     * Sweep each Group Set
     */
    {
      BLOCK_TIMER(user_data->timing, Sweep);
      for(int group_set = 0;group_set < user_data->num_group_sets;++ group_set){
        SweepSolver_GroupSet(group_set, user_data);
      }
    }
  }

  return(0);
}


/*----------------------------------------------------------------------
 * SweepSolverSolveDD
 *----------------------------------------------------------------------*/

int SweepSolver_GroupSet (int group_set, User_Data *user_data)
{
  Grid_Data  *grid_data         = user_data->grid_data;

  stapl::graph_view<Grid_Data> grid_view(*grid_data);

  // View used to sweep
  typedef stapl::result_of::native_view<stapl::graph_view<Grid_Data>>::type
    coarse_grid_view_type;

  typedef stapl::paragraph<stapl::default_scheduler,
            explicit_sweep_factory<Kernel_3d_DGZ>, coarse_grid_view_type>
    sweep_paragraph_type_DGZ;
  typedef stapl::paragraph<stapl::default_scheduler,
            explicit_sweep_factory<Kernel_3d_ZDG>, coarse_grid_view_type>
    sweep_paragraph_type_ZDG;
  typedef stapl::paragraph<stapl::default_scheduler,
            explicit_sweep_factory<Kernel_3d_GDZ>, coarse_grid_view_type>
    sweep_paragraph_type_GDZ;

  coarse_grid_view_type coarse_grid_view(stapl::native_view(grid_view));

  int num_direction_sets = user_data->num_direction_sets;

  /* Use standard Diamond-Difference sweep */
  {
    BLOCK_TIMER(user_data->timing, Sweep_Kernel);
    // Sweep in each direction set
    for (int ds = 0; ds < num_direction_sets; ++ds)
    {
      // Switch is needed to recover kernel type.
      // This is required because the function operator is templated, and a
      // virtual templated function calls aren't supported.
      switch(user_data->kernel->nestingPhi()){
        case NEST_DGZ:
        {
          Kernel_3d_DGZ* full_kernel =
            static_cast<Kernel_3d_DGZ*>(user_data->kernel);
          full_kernel->group_set = group_set;
          full_kernel->direction_set = ds;

          // Construct a PARAGRAPH for each direction set
          // PARAGRAPHs are heap allocated because their execution will continue
          // beyond the scope of the loop iteration.
          sweep_paragraph_type_DGZ* sweep = new sweep_paragraph_type_DGZ(
            explicit_sweep_factory<Kernel_3d_DGZ>(grid_data, *full_kernel),
            coarse_grid_view);

          // Add the PARAGRAPH to the executor and continue starting sweeps
          (*sweep)(0);
          break;
        }
        case NEST_DZG:
          printf("Nesting DZG not supported yet.\n");
          std::exit(1);
        case NEST_GDZ:
        {
          Kernel_3d_GDZ* full_kernel =
            static_cast<Kernel_3d_GDZ*>(user_data->kernel);
          full_kernel->group_set = group_set;
          full_kernel->direction_set = ds;

          // Construct a PARAGRAPH for each direction set
          // PARAGRAPHs are heap allocated because their execution will continue
          // beyond the scope of the loop iteration.
          sweep_paragraph_type_GDZ* sweep = new sweep_paragraph_type_GDZ(
            explicit_sweep_factory<Kernel_3d_GDZ>(grid_data, *full_kernel),
            coarse_grid_view);

          // Add the PARAGRAPH to the executor and continue starting sweeps
          (*sweep)(0);
          break;
        }
        case NEST_GZD:
          printf("Nesting GZD not supported yet.\n");
          std::exit(1);
        case NEST_ZDG:
        {
          Kernel_3d_ZDG* full_kernel =
            static_cast<Kernel_3d_ZDG*>(user_data->kernel);
          full_kernel->group_set = group_set;
          full_kernel->direction_set = ds;

          // Construct a PARAGRAPH for each direction set
          // PARAGRAPHs are heap allocated because their execution will continue
          // beyond the scope of the loop iteration.
          sweep_paragraph_type_ZDG* sweep = new sweep_paragraph_type_ZDG(
            explicit_sweep_factory<Kernel_3d_ZDG>(grid_data, *full_kernel),
            coarse_grid_view);

          // Add the PARAGRAPH to the executor and continue starting sweeps
          (*sweep)(0);
          break;
        }
        case NEST_ZGD:
          printf("Nesting ZGD not supported yet.\n");
          std::exit(1);
        case NEST_DNM:
          printf("Nesting DNM not supported yet.\n");
          std::exit(1);
        case NEST_NMD:
          printf("Nesting NMD not supported yet.\n");
          std::exit(1);
      }
    }
    // cause all PARAGRAPHs to be executed.
    // What is the proper way to ensure all PARAGRAPHs have been processed?
    stapl::exit_code blocker(EXIT_SUCCESS);
    blocker.wait();
  }

  return(0);
}

/*----------------------------------------------------------------------
 * CreateBufferInfo
 *----------------------------------------------------------------------*/

void CreateBufferInfo(User_Data *user_data)
{
  Grid_Data  *grid_data  = user_data->grid_data;

  int *nzones          = grid_data->nzones();
  int local_imax, local_jmax, local_kmax;
  int num_direction_sets = grid_data->gd_sets()[0].size();
  int len[6], nm[6], length;

  // get group and direction dimensionality
  int dirs_groups = user_data->num_directions_per_set
                  * user_data->num_groups_per_set;

  local_imax = nzones[0];
  local_jmax = nzones[1];
  local_kmax = nzones[2];

  /* Info for buffers used for messages sent in the x direction */
  length = local_jmax * local_kmax + 1;
  len[0] = len[1] = length * dirs_groups;

  /* Info for buffers used for messages sent in the y direction */
  length = local_imax * local_kmax + 1;
  len[2] = len[3] = length * dirs_groups;

  /* Info for buffers used for messages sent in the z direction */
  length = local_imax * local_jmax + 1;
  len[4] = len[5] = length * dirs_groups;

  for(int i=0; i<6; i++){
    nm[i] = 0;
  }

  for(int ds=0; ds<num_direction_sets; ds++){
    Directions *directions = grid_data->gd_sets()[0][ds].directions;
    if(directions[0].id > 0){
      nm[0]++;
    }
    else {nm[1]++; }
    if(directions[0].jd > 0){
      nm[2]++;
    }
    else {nm[3]++; }
    if(directions[0].kd > 0){
      nm[4]++;
    }
    else {nm[5]++; }
  }
}

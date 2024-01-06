/*--------------------------------------------------------------------------
 * Sweep-based solver routine.
 *--------------------------------------------------------------------------*/

#include <Kripke.h>
#include <Kripke/User_Data.h>
#include <Kripke/Kernel/Kernel_3d_DGZ.h>
#include <Kripke/Kernel/Kernel_3d_DZG.h>
#include <Kripke/Kernel/Kernel_3d_ZDG.h>
#include <Kripke/Kernel/Kernel_3d_ZGD.h>
#include <Kripke/Kernel/Kernel_3d_GDZ.h>
#include <Kripke/Kernel/Kernel_3d_GZD.h>
#include <stapl/skeletons/executors/execute.hpp>
#include <stapl/views/metadata/coarseners/null.hpp>
#include <stapl/skeletons/functional/wavefront.hpp>
#include <sweep_filters.hpp>
#include <vector>
#include <stdio.h>

#include "../sweep_filters.hpp"

#include <stapl/runtime/counter/probe.hpp>

/*----------------------------------------------------------------------
 * SweepSolverSolve
 *----------------------------------------------------------------------*/

int SweepSolver (User_Data *user_data)
{
  Kernel *kernel = user_data->kernel;

  BLOCK_TIMER(user_data->timing, Solve);

  // Loop over iterations
  for (int iter = 0; iter < user_data->niter; ++iter) {

    /*
     * Compute the RHS
     */

    // Discrete to Moments transformation
    {
      BLOCK_TIMER(user_data->timing, LTimes);
      kernel->LTimes();
    }

    // This is where the Scattering kernel would go!



    // Moments to Discrete transformation
    {
      BLOCK_TIMER(user_data->timing, LPlusTimes);
      kernel->LPlusTimes();
    }

    /*
     * Sweep each Group Set
     */

    {
      BLOCK_TIMER(user_data->timing, Sweep);
      for (int group_set = 0;
           group_set < user_data->num_group_sets;
           ++ group_set)
      {
        switch(user_data->kernel->nestingPhi()){
          case NEST_DGZ:
          {
            Kernel_3d_DGZ* full_kernel =
              static_cast<Kernel_3d_DGZ*>(user_data->kernel);
            SweepSolver_GroupSet(group_set, user_data, full_kernel);
            break;
          }
          case NEST_DZG:
          {
            Kernel_3d_DZG* full_kernel =
              static_cast<Kernel_3d_DZG*>(user_data->kernel);
            SweepSolver_GroupSet(group_set, user_data, full_kernel);
            break;
          }
          case NEST_GDZ:
          {
            Kernel_3d_GDZ* full_kernel =
              static_cast<Kernel_3d_GDZ*>(user_data->kernel);
            SweepSolver_GroupSet(group_set, user_data, full_kernel);
            break;
          }
          case NEST_GZD:
          {
            Kernel_3d_GZD* full_kernel =
              static_cast<Kernel_3d_GZD*>(user_data->kernel);
            SweepSolver_GroupSet(group_set, user_data, full_kernel);
            break;
          }
          case NEST_ZDG:
          {
            Kernel_3d_ZDG* full_kernel =
              static_cast<Kernel_3d_ZDG*>(user_data->kernel);
            SweepSolver_GroupSet(group_set, user_data, full_kernel);
            break;
          }
          case NEST_ZGD:
          {
            Kernel_3d_ZGD* full_kernel =
              static_cast<Kernel_3d_ZGD*>(user_data->kernel);
            SweepSolver_GroupSet(group_set, user_data, full_kernel);
            break;
          }
          case NEST_DNM:
            printf("Nesting DNM not supported yet.\n");
            std::exit(1);
          case NEST_NMD:
            printf("Nesting NMD not supported yet.\n");
            std::exit(1);
        }
      }
    }
  }

  return(0);
}

template <typename Kernel, typename Sigt, typename Psi, typename Rhs,
          typename IndexView, typename Bndry0, typename Bndry1, typename Bndry2>
void sweep(Grid_Data_Base* grid_data, Kernel* kernel, int group_set,
           int direction_set, Sigt&& sigt, Psi&& psi, Rhs&& rhs,
           IndexView&& index_view, Bndry0&& bndary0, Bndry1&& bndary1,
           Bndry2&& bndary2)
{
  using namespace stapl;
  using namespace stapl::skeletons;

  kernel->group_set = group_set;
  kernel->direction_set = direction_set;

  auto&& corner =
    find_corner(grid_data->gd_sets()[group_set][direction_set].directions[0]);

  auto&& diamond_op = make_functional_diamond_difference_wf(
    grid_data, psi, group_set, direction_set, std::make_tuple(0, 0, 0));

  auto kernel_sk =
    kernel_skeleton<Kernel::nesting_val>::make(corner, diamond_op);
  auto nested_kernel_sk = nest(kernel_sk);

  int num_groups = grid_data->gd_sets()[group_set][direction_set].num_groups;
  int num_dirs = grid_data->gd_sets()[group_set][direction_set].num_directions;

  auto&& num_zonesets = grid_data->num_zonesets();
  auto&& zone_size    = grid_data->zone_size();
  auto&& num_locs     = grid_data->num_locs();

  kernel_skeleton<Kernel::nesting_val>::set_level_dims(
    num_locs, num_zonesets, zone_size, num_groups, num_dirs, nested_kernel_sk);

  // null coarsener used because view elements are coarse, as is the
  // work function.
  skeletons::execute(
    skeletons::execution_params<void, true>(),
    nested_kernel_sk,
    *psi[group_set][direction_set],
    *rhs[group_set][direction_set],
    *sigt[group_set],
    *index_view[group_set][direction_set],
    *bndary0[group_set][direction_set],
    *bndary1[group_set][direction_set],
    *bndary2[group_set][direction_set]);
}

/*----------------------------------------------------------------------
 * SweepSolverSolveDD
 *----------------------------------------------------------------------*/

template <typename FullKernel>
int SweepSolver_GroupSet (int group_set, User_Data *user_data,
                          FullKernel* full_kernel)
{
  int num_direction_sets = user_data->num_direction_sets;
  /* Use standard Diamond-Difference sweep */
  {
    BLOCK_TIMER(user_data->timing, Sweep_Kernel);
    // Sweep in each direction set
    for (int ds = 0; ds < num_direction_sets; ++ds)
    {
      sweep(full_kernel->grid_data, full_kernel, group_set, ds,
            full_kernel->grid_data->sliced_sigt(),
            full_kernel->grid_data->sliced_psi(),
            full_kernel->grid_data->sliced_rhs(),
            full_kernel->grid_data->index_view(),
            full_kernel->grid_data->boundary0(),
            full_kernel->grid_data->boundary1(),
            full_kernel->grid_data->boundary2());
    }

    // cause all PARAGRAPHs to be executed
    stapl::get_executor()(stapl::execute_all);
  }

  return(0);
}

/*----------------------------------------------------------------------
 * CreateBufferInfo
 *----------------------------------------------------------------------*/

void CreateBufferInfo(User_Data *user_data)
{
  Grid_Data_Base* grid_data  = user_data->grid_data;

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

  for (int i=0; i<6; i++) {
    nm[i] = 0;
  }

  for (int ds=0; ds<num_direction_sets; ds++) {
    Directions *directions = grid_data->gd_sets()[0][ds].directions;
    if (directions[0].id > 0) {
      nm[0]++;
    }
    else {nm[1]++; }
    if (directions[0].jd > 0) {
      nm[2]++;
    }
    else {nm[3]++; }
    if (directions[0].kd > 0) {
      nm[4]++;
    }
    else {nm[5]++; }
  }
}

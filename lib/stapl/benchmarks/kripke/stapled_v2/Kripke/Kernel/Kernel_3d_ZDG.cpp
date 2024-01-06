#include<Kripke/Kernel/Kernel_3d_ZDG.h>
#include<Kripke/User_Data.h>
#include "kernel_work_functions.hpp"

Kernel_3d_ZDG::Kernel_3d_ZDG(Grid_Data_Base* grid)
  : Kernel(NEST_ZDG),
    grid_data(static_cast<Grid_Data<NEST_ZDG>*>(grid))
{ }

Kernel_3d_ZDG::~Kernel_3d_ZDG()
{ }


void Kernel_3d_ZDG::LTimes(void)
{
  // Dimension information for inner work functions
  int nidx = grid_data->nm_table().size();

  // loop over group sets
  int num_group_sets = grid_data->gd_sets().size();
  for (int gset = 0; gset < num_group_sets; gset++)
  {
    // loop over direction sets
    int num_direction_sets = grid_data->gd_sets()[gset].size();
    for (int dset = 0; dset < num_direction_sets; dset++)
    {
      // Compute L
      stapl::map_func(ltimes_wf(nidx, dset,
                        grid_data->gd_sets()[gset][dset].num_directions),
                      *grid_data->psi()[gset][dset],
                      *grid_data->psi_ltimes()[gset][dset],
                      *grid_data->ell()[dset],
                      *grid_data->phi()[gset]);
    }
  }
}


void Kernel_3d_ZDG::LPlusTimes(void)
{
  // Dimension information for inner work functions
  int nidx = grid_data->nm_table().size();

  // loop over group sets
  int num_group_sets = grid_data->gd_sets().size();
  for (int gset = 0; gset < num_group_sets; ++gset)
  {
    // loop over direction sets
    int num_direction_sets = grid_data->gd_sets()[gset].size();
    for (int dset = 0; dset < num_direction_sets; ++dset)
    {
      // Compute rhs
      stapl::map_func(lplustimes_wf(nidx,
                        grid_data->gd_sets()[gset][dset].num_directions),
                      *grid_data->rhs()[gset][dset],
                      *grid_data->rhs_ltimes()[gset][dset],
                      *grid_data->ell_plus()[dset],
                      *grid_data->phi_out()[gset]);
    }
  }
}

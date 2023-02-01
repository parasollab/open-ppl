#include<Kripke/Kernel/Kernel_3d_DGZ.h>
#include<Kripke/User_Data.h>
#include "kernel_work_functions.hpp"

Kernel_3d_DGZ::Kernel_3d_DGZ(Grid_Data* grid)
  : grid_data(grid)
{ }

Kernel_3d_DGZ::~Kernel_3d_DGZ() { }

Nesting_Order Kernel_3d_DGZ::nestingPsi(void) const {
  return NEST_DGZ;
}

Nesting_Order Kernel_3d_DGZ::nestingPhi(void) const {
  return NEST_DGZ;
}


struct dgz_ltimes_wf
{
private:
  Grid_Data* m_grid;

public:
  typedef void result_type;

  dgz_ltimes_wf(Grid_Data* grid)
    : m_grid(grid)
  { }

  template <typename Vertex>
  result_type operator()(Vertex v)
  {
    typedef std::array<typename Vertex::property_type::storage_type::index, 2>
      index_type;

    // Outer parameters
    int nidx = m_grid->nm_table().size();

    // Loop over Group Sets
    int num_group_sets = m_grid->gd_sets().size();
    for (int gset = 0; gset < num_group_sets; ++gset)
    {
      std::vector<Group_Dir_Set> &dir_sets = m_grid->gd_sets()[gset];
      int num_dir_sets = dir_sets.size();

      // Loop over Direction Sets
      for (int dset = 0; dset < num_dir_sets; ++dset)
      {
        Group_Dir_Set &gd_set = dir_sets[dset];

        // Get dimensioning
        int num_local_groups = gd_set.num_groups;
        int group0 = gd_set.group0;
        int num_local_directions = gd_set.num_directions;
        int dir0 = gd_set.direction0;

        index_type ell_idx{{0, 0}};
        index_type phi_idx{{0, 0}};
        index_type psi_idx{{0, 0}};

        for(int nm_offset = 0; nm_offset < nidx; ++nm_offset)
        {
          ell_idx[0] = nm_offset;
          phi_idx[1] = nm_offset;

          for (int d = 0; d < num_local_directions; d++)
          {
            ell_idx[1] = dir0 + d;
            psi_idx[1] = d;
            double ell_d_nm = v.property().ell()(ell_idx);

            // Used to be a parallel loop over group-zones
            for (int group = 0; group < num_local_groups; ++group)
            {
              phi_idx[0] = group0 + group;
              psi_idx[0] = group;
              v.property().phi()(phi_idx) +=
                ell_d_nm * v.property().psi()[gset][dset](psi_idx);
            }
          }
        }
      } // Direction Set
    } // Group Set
  }

  void define_type(stapl::typer& t)
  { t.member(m_grid); }
};

void Kernel_3d_DGZ::LTimes(Grid_Data *grid_data)
{
  // Construct pView over reference to prevent pView from taking ownership
  stapl::graph_view<Grid_Data> grid_view(*grid_data);

  // Clear phi
  stapl::map_func(
    clear_phi(0.0,
      grid_data->gd_sets().size()*grid_data->gd_sets()[0][0].num_groups,
      grid_data->nm_table().size()),
    grid_view);

  // Compute L
  stapl::map_func(dgz_ltimes_wf(grid_data), grid_view);
}


struct dgz_lplustimes_wf
{
private:
  Grid_Data* m_grid;

public:
  typedef void result_type;

  dgz_lplustimes_wf(Grid_Data* grid)
    : m_grid(grid)
  { }

  template <typename Vertex>
  result_type operator()(Vertex v)
  {
    typedef std::array<typename Vertex::property_type::storage_type::index, 2>
      index_type;

    // Outer parameters
    int nidx = m_grid->nm_table().size();
    int num_group_sets = m_grid->gd_sets().size();

    // Loop over Group Sets
    for (int gset = 0; gset < num_group_sets; ++gset)
    {
      std::vector<Group_Dir_Set> &dir_sets = m_grid->gd_sets()[gset];
      int num_dir_sets = dir_sets.size();

      // Loop over Direction Sets
      for (int dset = 0; dset < num_dir_sets; ++dset)
      {
        Group_Dir_Set &gd_set = dir_sets[dset];

        // Get dimensioning
        int num_local_groups = gd_set.num_groups;
        int group0 = gd_set.group0;
        int num_local_directions = gd_set.num_directions;
        int dir0 = gd_set.direction0;

        // Clear RHS
        index_type rhs_idx{{0,0}};
        for (int d = 0; d < num_local_directions; d++)
        {
          rhs_idx[1] = d;
          for (int group = 0; group < num_local_groups; ++group)
          {
            rhs_idx[0] = group;
            v.property().rhs()[gset][dset](rhs_idx) = 0.0;
          }
        }

        /* 3D Cartesian Geometry */
        index_type phi_out_idx{{0,0}};
        index_type ell_plus_idx{{0,0}};

        for (int d = 0; d < num_local_directions; d++)
        {
          rhs_idx[1] = d;
          ell_plus_idx[1] = dir0 + d;

          for(int nm_offset = 0; nm_offset < nidx; ++nm_offset)
          {
            phi_out_idx[1] = nm_offset;
            ell_plus_idx[0] = nm_offset;
            double ell_plus_d_n_m = v.property().ell_plus()(ell_plus_idx);

            // Used to be a parallel loop over group-zones
            for (int group = 0; group < num_local_groups; ++group)
            {
              rhs_idx[0] = group;
              phi_out_idx[0] = group0 + group;
              v.property().rhs()[gset][dset](rhs_idx) +=
                ell_plus_d_n_m * v.property().phi_out()(phi_out_idx);
            }
          }
        }
      } // Direction Set
    } // Group Set
  }

  void define_type(stapl::typer& t)
  { t.member(m_grid); }
};

void Kernel_3d_DGZ::LPlusTimes(Grid_Data *grid_data)
{
  stapl::graph_view<Grid_Data> grid_view(*grid_data);
  stapl::map_func(dgz_lplustimes_wf(grid_data), grid_view);
}

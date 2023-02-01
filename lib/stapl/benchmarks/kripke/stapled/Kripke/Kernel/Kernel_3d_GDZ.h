#ifndef KRIPKE_KERNEL_3D_GDZ_H__
#define KRIPKE_KERNEL_3D_GDZ_H__

#include<Kripke/Kernel.h>
#include<Grid.h>

class Kernel_3d_GDZ : public Kernel {
public:
  typedef std::vector<std::vector<double>> result_type;

  // Grid is needed to access metadata (e.g. gd_sets) stored on it.
  Grid_Data* grid_data;
  int group_set;
  int direction_set;

  Kernel_3d_GDZ(Grid_Data*);
  virtual ~Kernel_3d_GDZ();

  virtual Nesting_Order nestingPsi(void) const;
  virtual Nesting_Order nestingPhi(void) const;

  virtual void LTimes(Grid_Data *grid_data);
  virtual void LPlusTimes(Grid_Data *grid_data);

  template<typename GridView, typename IPlane, typename JPlane, typename KPlane>
  result_type
  operator()(GridView& grid_view, IPlane const& i_plane, JPlane const& j_plane,
             KPlane const& k_plane);

  void define_type(stapl::typer& t)
  {
    t.member(grid_data);
    t.member(group_set);
    t.member(direction_set);
  }
};


/* Sweep routine for Diamond-Difference */
/* Macros for offsets with fluxes on cell faces */
#define I_PLANE_INDEX(j, k) (k)*(local_jmax) + (j)
#define J_PLANE_INDEX(i, k) (k)*(local_imax) + (i)
#define K_PLANE_INDEX(i, j) (j)*(local_imax) + (i)
#define Zonal_INDEX(i, j, k) (i) + (local_imax)*(j) \
  + (local_imax)*(local_jmax)*(k)

template<typename GridView, typename IPlane, typename JPlane, typename KPlane>
std::vector<std::vector<double>>
Kernel_3d_GDZ::operator()(GridView& grid_view, IPlane const& i_plane_in,
                          JPlane const& j_plane_in, KPlane const& k_plane_in)
{
  typedef std::array<typename GridView::value_type::property_type::
                       storage_type::index, 2> index_type;

  result_type result(3);

  std::vector<double> i_plane = i_plane_in[0];
  std::vector<double> j_plane = j_plane_in[0];
  std::vector<double> k_plane = k_plane_in[0];

  // grid_data, group_set, and direction_set are data members of the Kernel.
  Group_Dir_Set& gd_set = grid_data->gd_sets()[group_set][direction_set];

  int num_directions = gd_set.num_directions;
  int num_groups = gd_set.num_groups;

  Directions *direction = gd_set.directions;

  int local_imax = grid_data->nzones()[0];
  int local_jmax = grid_data->nzones()[1];
  int local_kmax = grid_data->nzones()[2];

  auto dx = grid_data->deltas(0);
  auto dy = grid_data->deltas(1);
  auto dz = grid_data->deltas(2);

  // All directions have same id,jd,kd, since these are all one Direction Set
  // So pull that information out now
  int octant = direction[0].octant;
  Grid_Sweep_Block const &extent = grid_data->octant_extent()[octant];


#ifdef KRIPKE_USE_OPENMP
#pragma omp parallel for
#endif
  for (int group = 0; group < num_groups; ++group) {

    std::vector<double> xcos_dxi_all(local_imax);
    std::vector<double> ycos_dyj_all(local_jmax);
    std::vector<double> zcos_dzk_all(local_kmax);
    index_type sigt_idx{{gd_set.group0+group, 0}};

    for (int d = 0; d < num_directions; ++d) {
      index_type psi_idx{{group, d}};
      index_type rhs_idx{{group, d}};

      int plane_idx = num_directions * num_groups + d * num_groups + group;

      double xcos = direction[d].xcos;
      double ycos = direction[d].ycos;
      double zcos = direction[d].zcos;
      for (int i = 0; i < local_imax; ++i) {
        double dxi = dx[i + 1];
        xcos_dxi_all[i] = 2.0 * xcos / dxi;
      }

      for (int j = 0; j < local_jmax; ++j) {
        double dyj = dy[j + 1];
        ycos_dyj_all[j] = 2.0 * ycos / dyj;
      }

      for (int k = 0; k < local_kmax; ++k) {
        double dzk = dz[k + 1];
        zcos_dzk_all[k] = 2.0 * zcos / dzk;
      }


      /*  Perform transport sweep of the grid 1 cell at a time.   */
      for (int k = extent.start_k; k != extent.end_k; k += extent.inc_k) {
        double zcos_dzk = zcos_dzk_all[k];
        for (int j = extent.start_j; j != extent.end_j; j += extent.inc_j) {
          double ycos_dyj = ycos_dyj_all[j];
          for (int i = extent.start_i; i != extent.end_i; i += extent.inc_i) {
            double xcos_dxi = xcos_dxi_all[i];

            /* Calculate new zonal flux */
            // get a reference to the vertex being processed.
            int z = Zonal_INDEX(i, j, k);
            auto v = (*grid_view.find_vertex(z)).property();

            double psi_lf_g_d_z = i_plane[I_PLANE_INDEX(j, k) * plane_idx];
            double psi_fr_g_d_z = j_plane[J_PLANE_INDEX(i, k) * plane_idx];
            double psi_bo_g_d_z = k_plane[K_PLANE_INDEX(i, j) * plane_idx];

            auto psi_z = v.psi()[group_set][direction_set];
            auto rhs_z = v.rhs()[group_set][direction_set];

            double psi_g_d_z = (rhs_z(rhs_idx)
                + psi_lf_g_d_z * xcos_dxi
                + psi_fr_g_d_z * ycos_dyj
                + psi_bo_g_d_z * zcos_dzk)
                / (xcos_dxi + ycos_dyj + zcos_dzk
                    + v.sigt()(sigt_idx));

            psi_z(psi_idx) = psi_g_d_z;

            /* Apply diamond-difference relationships */
            psi_g_d_z *= 2.0;
            psi_lf_g_d_z = psi_g_d_z - psi_lf_g_d_z;
            psi_fr_g_d_z = psi_g_d_z - psi_fr_g_d_z;
            psi_bo_g_d_z = psi_g_d_z - psi_bo_g_d_z;
          }
        }
      }
    }
  }
  result[0] = std::move(i_plane);
  result[1] = std::move(j_plane);
  result[2] = std::move(k_plane);
  return result;
}



#endif

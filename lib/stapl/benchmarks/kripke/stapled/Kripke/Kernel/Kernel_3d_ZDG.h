#ifndef KRIPKE_KERNEL_3D_ZDG_H__
#define KRIPKE_KERNEL_3D_ZDG_H__

#include<Kripke/Kernel.h>
#include<Grid.h>

class Kernel_3d_ZDG : public Kernel {
public:
  typedef std::vector<std::vector<double>> result_type;

  // Grid is needed to access metadata (e.g. gd_sets) stored on it.
  Grid_Data* grid_data;
  int group_set;
  int direction_set;

  Kernel_3d_ZDG(Grid_Data*);
  virtual ~Kernel_3d_ZDG();

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
Kernel_3d_ZDG::operator()(GridView& grid_view, IPlane const& i_plane_in,
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

// TGS : compiler detects unused variable.  Are the macros correct?
//  int local_kmax = grid_data->nzones()[2];

  auto dx = grid_data->deltas(0);
  auto dy = grid_data->deltas(1);
  auto dz = grid_data->deltas(2);

  // All directions have same id,jd,kd, since these are all one Direction Set
  // So pull that information out now
  int octant = direction[0].octant;
  Grid_Sweep_Block const &extent = grid_data->octant_extent()[octant];

  for (int k = extent.start_k; k != extent.end_k; k += extent.inc_k) {
    double dzk = dz[k + 1];
    for (int j = extent.start_j; j != extent.end_j; j += extent.inc_j) {
      double dyj = dy[j + 1];
      for (int i = extent.start_i; i != extent.end_i; i += extent.inc_i) {
        double dxi = dx[i + 1];

        // get a reference to the vertex being processed.
        int z = Zonal_INDEX(i, j, k);
        auto v = (*grid_view.find_vertex(z)).property();

#ifdef KRIPKE_USE_OPENMP
#pragma omp parallel for
#endif
        for (int d = 0; d < num_directions; ++d) {
          double xcos = direction[d].xcos;
          double ycos = direction[d].ycos;
          double zcos = direction[d].zcos;

          double zcos_dzk = 2.0 * zcos / dzk;
          double ycos_dyj = 2.0 * ycos / dyj;
          double xcos_dxi = 2.0 * xcos / dxi;

          auto psi_z = v.psi()[group_set][direction_set];
          index_type psi_z_idx{{0, d}};

          auto rhs_z = v.rhs()[group_set][direction_set];
          index_type rhs_z_idx{{0, d}};


          int i_plane_idx = I_PLANE_INDEX(j, k)*num_directions*num_groups +
            d*num_groups;
          double * psi_lf_z_d = &i_plane[i_plane_idx];

          int j_plane_idx = J_PLANE_INDEX(i, k)*num_directions*num_groups +
            d*num_groups;
          double * psi_fr_z_d = &j_plane[j_plane_idx];

          int k_plane_idx = K_PLANE_INDEX(i, j)*num_directions*num_groups +
            d*num_groups;
          double * psi_bo_z_d = &k_plane[k_plane_idx];

          for (int group = 0; group < num_groups; ++group) {
            /* Calculate new zonal flux */
            index_type sigt_idx{{gd_set.group0+group, 0}};
            rhs_z_idx[0] = group;
            double psi_z_d_g = (rhs_z(rhs_z_idx)
                + psi_lf_z_d[group] * xcos_dxi
                + psi_fr_z_d[group] * ycos_dyj
                + psi_bo_z_d[group] * zcos_dzk)
                / (xcos_dxi + ycos_dyj + zcos_dzk +
                   v.sigt()(sigt_idx));
            psi_z_idx[0] = group;
            psi_z(psi_z_idx) = psi_z_d_g;

            /* Apply diamond-difference relationships */
            psi_z_d_g *= 2.0;
            psi_lf_z_d[group] = psi_z_d_g - psi_lf_z_d[group];
            psi_fr_z_d[group] = psi_z_d_g - psi_fr_z_d[group];
            psi_bo_z_d[group] = psi_z_d_g - psi_bo_z_d[group];
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

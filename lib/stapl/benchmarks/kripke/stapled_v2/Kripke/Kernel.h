/*--------------------------------------------------------------------------
 * Header file for the Grid_Data data structures
 *--------------------------------------------------------------------------*/

#ifndef KRIPKE_KERNEL_H__
#define KRIPKE_KERNEL_H__

#include <Kripke.h>
#include <Grid.h>
#include <Kripke/User_Data.h>

#include <Kripke/Kernel/kernel_work_functions.hpp>

#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>
#include <stapl/algorithm.hpp>
#include <stapl/utility/tuple/print.hpp>

struct Group_Dir_Set;

/**
 * This is the Kernel base-class and interface definition.
 * This abstracts the storage of Psi, Phi, L, L+ from the rest of the code,
 * providing data-layout specific routines.
 */
class Kernel
{
protected:
  typedef std::array<direction, 3> incoming_dirs_type;

  incoming_dirs_type                          m_incoming_dirs;
  Nesting_Order                               nesting_order;

public:
  int group_set;
  int direction_set;

  typedef std::array<std::vector<double>, 3> result_type;

  Kernel(Nesting_Order);

  virtual ~Kernel();

  Nesting_Order nestingPsi(void) const;
  Nesting_Order nestingPhi(void) const;

  // Variable Creation
  template<Nesting_Order>
  void allocateStorage(User_Data *user_data);

  // Computational Kernels
  virtual void LTimes(void)     = 0;
  virtual void LPlusTimes(void) = 0;

  void set_directions(incoming_dirs_type const& incoming_dirs);

  void define_type(stapl::typer& t)
  {
    t.member(group_set);
    t.member(direction_set);
    t.member(nesting_order);
  }
}; // class Kernel


// Factory to create correct kernel object
Kernel *createKernel(Nesting_Order, int num_dims, Grid_Data_Base* grid);


template<typename Spec1, typename Spec2,
         typename Traversal1, typename Traversal2, typename PartitionDimensions>
class get_spec_wf
{
private:
  typedef stapl::dist_view_impl::system_container   system_ct_t;
  typedef std::vector<system_ct_t>                  location_groups_t;

  typedef typename Spec1::gid_type gid1_type;
  typedef typename Spec2::gid_type gid2_type;

  gid1_type                           m_size1;
  gid2_type                           m_size2;
  gid1_type                           m_loc_dims;

  std::shared_ptr<location_groups_t>  m_loc_groups_ptr;

  void initialize_loc_groups()
  {
    const unsigned int num_loc_groups =
      stapl::get<0>(m_size1) * stapl::get<1>(m_size1) * stapl::get<2>(m_size1);

    const unsigned int num_locs = stapl::get_num_locations();

    const unsigned int loc_group_size = num_locs / num_loc_groups;

#ifndef STAPL_NDEBUG
    auto t = stapl::dist_spec_impl::modulo(m_loc_dims, m_size1);
    bool b = stapl::vs_map_reduce([](unsigned int val) { return val == 0; },
                                  stapl::logical_and<bool>(), true, t);

    stapl_assert(b, "Non Conformable location / cellset dims");
#endif

    for (unsigned int idx = 0; idx < num_loc_groups; ++idx)
    {
      std::vector<stapl::location_type> locs;

      unsigned int loc = idx * loc_group_size;

      for ( ; loc < loc_group_size * (idx + 1) ; ++loc)
      {
        locs.push_back(loc);
      }
      m_loc_groups_ptr->emplace_back(locs);
    }

  }

public:
   get_spec_wf(gid1_type size1, gid2_type size2, gid1_type loc_dims)
     : m_size1(size1), m_size2(size2),
       m_loc_dims(loc_dims),
       m_loc_groups_ptr(std::make_shared<location_groups_t>())
   { initialize_loc_groups(); }

   // Query for Top Level Container
   Spec1 operator()(void) const
   {
     return stapl::uniform_nd<Traversal1>(m_size1, m_loc_dims);
   }

   // Query for Second Level Container
   Spec2 operator()(typename Spec1::gid_type gid) const
   {
     const unsigned int loc_group =
       stapl::nd_linearize<
         typename Spec1::gid_type, stapl::default_traversal<3>::type
       >(m_size1)(gid);

     // Use Gid to define the set of locs on which nested container
     // is initialized.
     typedef PartitionDimensions part_dims;

     return stapl::sliced_volumetric<Traversal2, part_dims>(
       m_size2, m_loc_groups_ptr->operator[](loc_group));
   }
};


/**
 * Allocates kernel-specific storage for the User_Data object.
 */
template <Nesting_Order Nesting>
void Kernel::allocateStorage(User_Data *user_data)
{
  typedef typename Grid_Data<Nesting>::variable_types variable_types;

  using bndary_traits0 = grid_boundary_traits<0, Nesting>;
  using bndary_traits1 = grid_boundary_traits<1, Nesting>;
  using bndary_traits2 = grid_boundary_traits<2, Nesting>;

  using index_traits   = index_view_traits<Nesting>;

  // Inner Multiarray Related Typedefs
  typedef typename variable_types::D5_traversal    D5_traversal;
  typedef typename variable_types::D5_spec         D5_spec;
  typedef typename variable_types::zoneset_cont_t  zoneset_cont_t;

  typedef typename variable_types::D5_ltimes_traversal    D5_ltimes_traversal;
  typedef typename variable_types::D5_ltimes_spec         D5_ltimes_spec;
  typedef typename variable_types::ltimes_zoneset_cont_t  ltimes_zoneset_cont_t;

  // Outer Multiarray Related Typedefs
  typedef typename variable_types::D3_traversal           D3_traversal;
  typedef typename variable_types::D3_spec                D3_spec;
  typedef typename variable_types::variable_cont_t        variable_cont_t;
  typedef typename variable_types::variable_type          variable_type;
  typedef typename variable_types::slices_variable_type   slices_variable_type;
  typedef typename variable_types::sigt_slices_variable_type
    sigt_slices_variable_type;

  typedef typename index_traits::index_view_t          index_view_t;
  typedef typename index_traits::index_cont_t          index_cont_t;

  typedef
    typename variable_types::ltimes_variable_cont_t ltimes_variable_cont_t;
  typedef
    typename variable_types::ltimes_variable_type   ltimes_variable_type;

  // Specification generator
  typedef get_spec_wf<D3_spec, D5_spec, D3_traversal, D5_traversal,
    typename variable_types::partition_dims> spec_gen_t;

  typedef get_spec_wf<D3_spec, D5_ltimes_spec, D3_traversal,
    D5_ltimes_traversal, typename variable_types::ltimes_partition_dims>
    ltimes_spec_gen_t;

  Grid_Data<Nesting> *grid_data =
    static_cast<Grid_Data<Nesting>*>(user_data->grid_data);

  // Size of multiarray is the number of cellsets
  auto num_zonesets =
    std::make_tuple(grid_data->cx(), grid_data->cy(), grid_data->cz());

  // Location layout
  auto loc_size =
    std::make_tuple(grid_data->px(), grid_data->py(), grid_data->pz());

  // Size of each zone set
  auto zoneset_size =
    std::make_tuple(grid_data->ax(), grid_data->ay(), grid_data->az());

  auto outer_ct_size = num_zonesets;

  // The sizes of cellsets are uniform
  // Indexing of inner multiarrays is always (i, j, k, d, g)
  // This differs from Kripke SubTVec which is indexed as (group, moment, zone)

  // rhs and psi are stored on a per group/direction set basis.
  grid_data->psi().resize(grid_data->gd_sets().size());
  grid_data->psi_ltimes().resize(grid_data->gd_sets().size());
  grid_data->rhs().resize(grid_data->gd_sets().size());
  grid_data->rhs_ltimes().resize(grid_data->gd_sets().size());

  // sigt is stored on a per group set basis
  grid_data->sigt().resize(grid_data->gd_sets().size());

  grid_data->sliced_psi().resize(grid_data->gd_sets().size());
  grid_data->sliced_rhs().resize(grid_data->gd_sets().size());
  grid_data->sliced_sigt().resize(grid_data->gd_sets().size());

  // phi and phi_out are stored on a per group set basis
  grid_data->phi().resize(grid_data->gd_sets().size());
  grid_data->phi_out().resize(grid_data->gd_sets().size());

  grid_data->boundary0().resize(grid_data->gd_sets().size());
  grid_data->boundary1().resize(grid_data->gd_sets().size());
  grid_data->boundary2().resize(grid_data->gd_sets().size());

  // ell and ell_plus are stored on a per direction set basis
  grid_data->ell().resize(grid_data->gd_sets()[0].size());
  grid_data->ell_plus().resize(grid_data->gd_sets()[0].size());

  grid_data->index_view().resize(grid_data->gd_sets().size());

  // Allocate moments variables
  int num_moments = grid_data->num_moments();

  // Allocate L, L+ table nm indicies which make threading easier
  grid_data->nm_table().clear();
  for (int n = 0; n < num_moments; n++) {
    for (int m = -n; m <= n; m++) {
      grid_data->nm_table().push_back(n);
    }
  }

  int total_moments = grid_data->nm_table().size();

  for (size_t gs = 0;gs < grid_data->gd_sets().size();++ gs) {
    grid_data->psi()[gs].resize(grid_data->gd_sets()[gs].size(), nullptr);
    grid_data->psi_ltimes()[gs].resize(grid_data->gd_sets()[gs].size(),
                                       nullptr);
    grid_data->rhs()[gs].resize(grid_data->gd_sets()[gs].size(), nullptr);
    grid_data->rhs_ltimes()[gs].resize(grid_data->gd_sets()[gs].size(),
                                       nullptr);

    grid_data->sliced_psi()[gs].resize(grid_data->gd_sets()[gs].size(),
                                       nullptr);
    grid_data->sliced_rhs()[gs].resize(grid_data->gd_sets()[gs].size(),
                                       nullptr);

    grid_data->boundary0()[gs].resize(grid_data->gd_sets()[gs].size(), nullptr);
    grid_data->boundary1()[gs].resize(grid_data->gd_sets()[gs].size(), nullptr);
    grid_data->boundary2()[gs].resize(grid_data->gd_sets()[gs].size(), nullptr);

    grid_data->index_view()[gs].resize(grid_data->gd_sets()[gs].size(),
                                       nullptr);

    for (size_t ds = 0;ds < grid_data->gd_sets()[gs].size();++ds) {
      Group_Dir_Set &gdset = grid_data->gd_sets()[gs][ds];

      auto inner_ct_size = std::tuple_cat(zoneset_size,
        std::make_tuple(gdset.num_directions, gdset.num_groups));

      auto composed_spec =
        stapl::make_heterogeneous_composed_dist_spec<
          spec_gen_t, D3_spec, D5_spec>(spec_gen_t(outer_ct_size,
            inner_ct_size, loc_size));

      grid_data->psi()[gs][ds] =
        new variable_type(new variable_cont_t(composed_spec));

      // ensure all containers are allocated before moving to next multiarray
      stapl::rmi_fence();

      grid_data->rhs()[gs][ds] =
        new variable_type(new variable_cont_t(composed_spec));

      // ensure all containers are allocated before moving to next multiarray
      stapl::rmi_fence();

      grid_data->sliced_rhs()[gs][ds] =
        new slices_variable_type(grid_data->rhs()[gs][ds]->container());

      // ensure all containers are allocated before moving to next multiarray
      stapl::rmi_fence();

      grid_data->sliced_psi()[gs][ds] =
        new slices_variable_type(grid_data->psi()[gs][ds]->container());

      // ensure all containers are allocated before moving to next multiarray
      stapl::rmi_fence();

      grid_data->boundary0()[gs][ds] =
        bndary_traits0::make_view(loc_size, num_zonesets, zoneset_size,
                                  gdset.num_groups, gdset.num_directions);

      // ensure all containers are allocated before moving to next multiarray
      stapl::rmi_fence();

      grid_data->boundary1()[gs][ds] =
        bndary_traits1::make_view(loc_size, num_zonesets, zoneset_size,
                                  gdset.num_groups, gdset.num_directions);

      // ensure all containers are allocated before moving to next multiarray
      stapl::rmi_fence();

      grid_data->boundary2()[gs][ds] =
        bndary_traits2::make_view(loc_size, num_zonesets, zoneset_size,
                                  gdset.num_groups, gdset.num_directions);

      // ensure all containers are allocated before moving to next multiarray
      stapl::rmi_fence();

      grid_data->index_view()[gs][ds] =
        index_traits::make_view(loc_size, num_zonesets, zoneset_size,
                                gdset.num_groups, gdset.num_directions);

      // ensure all containers are allocated before moving to next multiarray
      stapl::rmi_fence();

      auto composed_ltimes_spec =
        stapl::make_heterogeneous_composed_dist_spec<
          ltimes_spec_gen_t, D3_spec, D5_ltimes_spec>(
            ltimes_spec_gen_t(outer_ct_size, inner_ct_size, loc_size));

      grid_data->psi_ltimes()[gs][ds] =
        new ltimes_variable_type(
          new ltimes_variable_cont_t(composed_ltimes_spec));

      // ensure all containers are allocated before moving to next multiarray
      stapl::rmi_fence();

      grid_data->rhs_ltimes()[gs][ds] =
        new ltimes_variable_type(
          new ltimes_variable_cont_t(composed_ltimes_spec));

      // ensure all containers are allocated before moving to next multiarray
      stapl::rmi_fence();

      if (gs == 0) {
        // ell and ell_plus are 1 * num_directions * total_moments in reference.
        // i,j,k domains are all size 1 to accomodate this without changing
        // type.
        auto ell_inner_size =
          std::make_tuple(1, 1, 1, gdset.num_directions, total_moments);

        auto ell_comp_spec = stapl::make_heterogeneous_composed_dist_spec<
          ltimes_spec_gen_t, D3_spec, D5_ltimes_spec>(
            ltimes_spec_gen_t(outer_ct_size, ell_inner_size, loc_size));

        grid_data->ell()[ds] =
          new ltimes_variable_type(new ltimes_variable_cont_t(ell_comp_spec));

        // ensure all containers are allocated before moving to next multiarray
        stapl::rmi_fence();

        grid_data->ell_plus()[ds] =
          new ltimes_variable_type(new ltimes_variable_cont_t(ell_comp_spec));

        // ensure all containers are allocated before moving to next multiarray
        stapl::rmi_fence();
      }
    }

    // sigt's lack of elements in the direction dimension is currently handled
    // by an extended_view that extends the view's domain so it is conformable
    // with psi and rhs. This doesn't change the distribution of the container.
    // For the DGZ nesting sweep, we attempt to slice this artificial dimension,
    // which results in an abort in the multiarray proxy when we attempt to get
    // a slice of the container for the sweep.
    // The workaround here is to artificially extend sigt so the code will run.
    // Alternatives to this that are being explored are:
    // 1. Recognize when we attempt to slice the extended dimension, and push
    //    the container into immutable_shared memory so all locations on the
    //    node can access it.
    //
    // 2. On-demand copy of the container on the first read.
    int dgz_sigt_dim_fix = 1;
    if (Nesting == NEST_DGZ or Nesting == NEST_DZG)
      dgz_sigt_dim_fix = grid_data->gd_sets()[gs][0].num_directions;

    // sigt is num_zones * 1 * num_groups
    auto inner_ct_size =
      std::tuple_cat(zoneset_size,
                     std::make_tuple(dgz_sigt_dim_fix,
                                     grid_data->gd_sets()[gs][0].num_groups));

    auto sigt_comp_spec =
      stapl::make_heterogeneous_composed_dist_spec<
        spec_gen_t, D3_spec, D5_spec>(spec_gen_t(outer_ct_size,
          inner_ct_size, loc_size));

    grid_data->sigt()[gs] =
      new variable_type(new variable_cont_t(sigt_comp_spec));

    // ensure all containers are allocated before exiting the function
    stapl::rmi_fence();

    grid_data->sliced_sigt()[gs] =
      new sigt_slices_variable_type(grid_data->sigt()[gs]->container(),
                                    grid_data->gd_sets()[gs][0].num_directions);

    // ensure all containers are allocated before exiting the function
    stapl::rmi_fence();

    // phi and phi_out are num_zones * num_groups * total_moments.
    auto phi_inner_size = std::tuple_cat(zoneset_size,
      std::make_tuple(total_moments, grid_data->gd_sets()[gs][0].num_groups));

    auto phi_comp_spec = stapl::make_heterogeneous_composed_dist_spec<
      ltimes_spec_gen_t, D3_spec, D5_ltimes_spec>(
        ltimes_spec_gen_t(outer_ct_size, phi_inner_size, loc_size));

    // Initialize the variables with views that own the underlying multiarray
    // container.
    grid_data->phi()[gs] =
      new ltimes_variable_type(new ltimes_variable_cont_t(phi_comp_spec));

    // ensure all containers are allocated before moving to next multiarray
    stapl::rmi_fence();

    grid_data->phi_out()[gs] =
      new ltimes_variable_type(new ltimes_variable_cont_t(phi_comp_spec));

    // ensure all containers are allocated before moving to next multiarray
    stapl::rmi_fence();
  }
}

#endif

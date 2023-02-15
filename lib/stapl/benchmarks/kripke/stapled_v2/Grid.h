/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#ifndef STAPL_BENCHMARKS_KRIPKE_GRID3D_H
#define STAPL_BENCHMARKS_KRIPKE_GRID3D_H

// System includes
#include <iostream>
#include <vector>

#include "GroupDirSet.hpp"
#include "Boundaries/Kernel_3d_DGZ.h"
#include "Boundaries/index_view_DGZ.h"
#include "Boundaries/Kernel_3d_DZG.h"
#include "Boundaries/index_view_DZG.h"
#include "Boundaries/Kernel_3d_GDZ.h"
#include "Boundaries/index_view_GDZ.h"
#include "Boundaries/Kernel_3d_GZD.h"
#include "Boundaries/index_view_GZD.h"
#include "Boundaries/Kernel_3d_ZDG.h"
#include "Boundaries/index_view_ZDG.h"
#include "Boundaries/Kernel_3d_ZGD.h"
#include "Boundaries/index_view_ZGD.h"

#include "Kripke/Input_Variables.h"

#include <stapl/views/multilevel_slices_view.hpp>
#include "sigt_view.hpp"
#include <stapl/views/counting_view.hpp>

#include <stapl/multiarray.hpp>
#include <stapl/skeletons/map.hpp>
#include <stapl/skeletons/map_reduce.hpp>

template <Nesting_Order, typename Container, typename InnerSlice,
          typename OuterSlice>
struct sigt_view_type
{
  using type = stapl::sigt_view<Container, InnerSlice, OuterSlice>;
};

template <typename Container, typename InnerSlice, typename OuterSlice>
struct sigt_view_type<NEST_DGZ, Container, InnerSlice, OuterSlice>
{
  using type =
    stapl::multilevel_slices_view<Container, InnerSlice, OuterSlice>;
};

template <typename Container, typename InnerSlice, typename OuterSlice>
struct sigt_view_type<NEST_DZG, Container, InnerSlice, OuterSlice>
{
  using type =
    stapl::multilevel_slices_view<Container, InnerSlice, OuterSlice>;
};

template <Nesting_Order nesting, typename Traversal,
          typename PartitionDimensions, typename InnerSlice,
          typename OuterSlice>
struct multiarray_types
{
  /// @brief Defines order in which elements of the container are linearized.
  using D5_traversal = Traversal;

  using D5_ltimes_traversal = stapl::index_sequence<3, 2, 1, 0, 4>;

  /// @brief Defines the dimensions of the inner containers that will be
  /// partitioned by their distribution.
  using partition_dims = PartitionDimensions;

  /// @brief Defines the dimensions of the inner containers that will be
  /// partitioned by their distribution in the LTimes/LPlusTimes stages.
  using ltimes_partition_dims = stapl::index_sequence<0,1,2,4>;

  /// @brief Type of the distribution specification that will distribute the
  /// container in a balaced manner along the three dimensions that make up the
  /// spatial domain.
  using D5_spec = typename stapl::sliced_md_distribution_spec<
                             partition_dims, D5_traversal>::type;

  using D5_ltimes_spec = typename stapl::sliced_md_distribution_spec<
                           ltimes_partition_dims, D5_ltimes_traversal>::type;

  /// @brief Defines order in which elements of the outer container are
  /// linearized.
  using D3_traversal = stapl::index_sequence<2, 1, 0>;

  /// @brief Type of the distribution specification that will distribute the
  /// container in a uniform manner along all three of its dimensions.
  using D3_spec = stapl::md_distribution_spec<D3_traversal>::type;

  /// @brief Type of the container representing a zoneset used in the sweep.
  /// The 5 dimensions represent space, direction, and energy. The specification
  /// includes a distribution that is based on the nesting used.
  using zoneset_cont_t = stapl::multiarray<5, double, D5_spec>;

  /// @brief Type of the container storing the collection of zonesets used in
  /// the sweep. The 3 dimensions represent the spatial dimensions, as a
  /// zoneset is a hierarchical representation of the spatial domain.
  using variable_cont_t = stapl::multiarray<3, zoneset_cont_t, D3_spec>;

  /// @brief Type of the view over the container of zonesets used in the sweep.
  /// This view type is used for the variables psi, rhs, and sigt in the Grid.
  using variable_type = stapl::multiarray_view<variable_cont_t>;

  using slices_variable_type =
    stapl::multilevel_slices_view<variable_cont_t, InnerSlice, OuterSlice>;

  using sigt_slices_variable_type =
    typename sigt_view_type<nesting, variable_cont_t, InnerSlice,
                            OuterSlice>::type;

  /// @brief Type of the container representing a zoneset used in LTimes and
  /// LPlusTimes. The 5 dimensions represent space, direction, and energy. The
  /// specification includes a distribution across energy and spatial
  /// dimensions.
  using ltimes_zoneset_cont_t = stapl::multiarray<5, double, D5_ltimes_spec>;

  /// @brief Type of the container storing the collection of zonesets used in
  /// the LTimes and LPlusTimes computations. The 3 dimensions represent the
  /// spatial dimensions, as a zoneset is a hierarchical representation of the
  /// spatial domain.
  using ltimes_variable_cont_t =
          stapl::multiarray<3, ltimes_zoneset_cont_t, D3_spec>;

  /// @brief Type of the view over the container of zonesets. This view type is
  /// used for all LTimes and LPlusTimes computations on the variables
  /// psi_ltimes, rhs_ltimes, ell, ell_plus, phi, and phi_out in the Grid.
  using ltimes_variable_type = stapl::multiarray_view<ltimes_variable_cont_t>;
};


//////////////////////////////////////////////////////////////////////
/// @brief Define the types of the multiarray instances used for the grid
/// variables.
///
/// Each Grid variable is a composed multiarray instance.  The outer container
/// has a single element for each zoneset.  The inner containers represent the
/// phase space that includes the zones in the zoneset and all points in the
/// energy and space domain.
///
/// The default layout of the container is the ZDG layout as it matches the
/// order in which we index the multiarray (i.e., a(i, j, k, d, g)).
///
/// Traversal order is specified in reverse order of what is expected given
/// the nesting. Recall that the dimensions of multiarray are (x,y,z,d,g).
///
//////////////////////////////////////////////////////////////////////
template<Nesting_Order>
struct grid_variable_traits;

template<>
struct grid_variable_traits<NEST_ZGD>
  : public multiarray_types<
       NEST_ZGD,
       stapl::index_sequence<4, 3, 2, 0, 1>,
       stapl::index_sequence<0, 1, 2>,
       stapl::index_sequence<0, 1, 2, 4>,
       stapl::index_sequence<0, 1, 2>>
{ };


template<>
struct grid_variable_traits<NEST_ZDG>
  : public multiarray_types<
      NEST_ZDG,
      stapl::index_sequence<4, 3, 2, 1, 0>,
      stapl::index_sequence<0, 1, 2>,
      stapl::index_sequence<0, 1, 2, 3>,
      stapl::index_sequence<0, 1, 2>>
{ };


template<>
struct grid_variable_traits<NEST_DZG>
  : public multiarray_types<
      NEST_DZG,
      stapl::index_sequence<3, 2, 1, 4, 0>,
      stapl::index_sequence<3>,
      stapl::index_sequence<0, 1, 2, 3>,
      stapl::index_sequence<3>>
{ };


template<>
struct grid_variable_traits<NEST_GZD>
  : public multiarray_types<
      NEST_GZD,
      stapl::index_sequence<3, 2, 1, 0, 4>,
      stapl::index_sequence<4>,
      stapl::index_sequence<0, 1, 2, 4>,
      stapl::index_sequence<3>>
{ };


template<>
struct grid_variable_traits<NEST_DGZ>
  : public multiarray_types<
      NEST_DGZ,
      stapl::index_sequence<2, 1, 0, 4, 3>,
      stapl::index_sequence<3>,
      stapl::index_sequence<3, 4>,
      stapl::index_sequence<0>>
{ };


template<>
struct grid_variable_traits<NEST_GDZ>
  : public multiarray_types<
      NEST_GDZ,
      stapl::index_sequence<2, 1, 0, 3, 4>,
      stapl::index_sequence<4>,
      stapl::index_sequence<3, 4>,
      stapl::index_sequence<1>>
{ };


class Grid_Data_Base : public stapl::p_object
{
protected:
  int m_px, m_py, m_pz;

  // Number of zones in each dimension of a zoneset
  int m_ax, m_ay, m_az;

  // Number of zonesets in each dimension for the entire spatial domain
  int m_cx, m_cy, m_cz;

  // Total Number of zones in the grid on this location
  int m_num_zones;

  // Number of zones in each dimension on this location
  int m_nzones[3];

  // Neighboring locations in each dimension
  int m_mynbr[3][2];

  // Spatial grid deltas in each dimension
  // Constructed in computeGrid.
  std::vector<double> m_deltas[3];

  // Sweep index sets for each octant
  // Computed in computeSweepIndexSets
  std::vector<Grid_Sweep_Block> m_octant_extent;

  // Group/Angle sets
  std::vector<std::vector<Group_Dir_Set> > m_gd_sets;

  int m_num_moments;

  // n, m indicies for traversing ell, ell_plus of vertices
  std::vector<int> m_nm_table;

  // Store nesting value to make it available in contexts where input_variables
  // isn't available.
  Nesting_Order m_nesting;

  void computeSweepIndexSets(void);

public:
  /*!
    \brief Build a grid from already parsed data.
  */
  Grid_Data_Base(Input_Variables *input_vars, Directions *directions);

  auto num_locs(void) const
    -> decltype(std::make_tuple(m_px, m_py, m_pz))
  {
    return std::make_tuple(m_px, m_py, m_pz);
  }

  auto zone_size(void) const
    -> decltype(std::make_tuple(m_ax, m_ay, m_az))
  {
    return std::make_tuple(m_ax, m_ay, m_az);
  }

  auto num_zonesets(void) const
    -> decltype(std::make_tuple(m_cx, m_cy, m_cz))
  {
    return std::make_tuple(m_cx, m_cy, m_cz);
  }

  int num_zones(void) const
  { return m_num_zones; }

  int px(void) const
  { return m_px; }

  int py(void) const
  { return m_py; }

  int pz(void) const
  { return m_pz; }

  int ax(void) const
  { return m_ax; }

  int ay(void) const
  { return m_ay; }

  int az(void) const
  { return m_az; }

  int cx(void) const
  { return m_cx; }

  int cy(void) const
  { return m_cy; }

  int cz(void) const
  { return m_cz; }

  int nzones(int dim) const
  { return m_nzones[dim]; }

  int (&nzones(void))[3]
  { return m_nzones; }

  int (&mynbr(void))[3][2]
  { return m_mynbr; }

  std::vector<double> const& deltas(int dim) const
  { return m_deltas[dim]; }

  std::vector<Grid_Sweep_Block> const& octant_extent(void) const
  { return m_octant_extent; }

  std::vector<std::vector<Group_Dir_Set> >& gd_sets(void)
  { return m_gd_sets; }

  std::vector<std::vector<Group_Dir_Set> > const& gd_sets(void) const
  { return m_gd_sets; }

  int num_moments(void) const
  { return m_num_moments; }

  std::vector<int> const& nm_table(void) const
  { return m_nm_table; }

  std::vector<int>& nm_table(void)
  { return m_nm_table; }

  Nesting_Order nesting(void) const
  { return m_nesting; }

  void computeGrid(int dim, int num_locs, int num_zones,
         stapl::location_type loc_index, double min_coord, double max_coord);

  virtual void randomizeData(void) = 0;

  virtual void copy(Grid_Data_Base const&) = 0;

  virtual bool compare(Grid_Data_Base const&, double, bool) = 0;
};


////////////////////////////////////////////////////////////////////////////////
///
/// @brief Represents a hexahedral grid.
///
/// @tparam VariableTraits Traits class specifying traversal orders and
/// distribution specifications for the multiarrays that are data members of
/// the class.
///
////////////////////////////////////////////////////////////////////////////////
template <Nesting_Order Nest>
class Grid_Data : public Grid_Data_Base
{
public:
  using variable_types           = grid_variable_traits<Nest>;

  using ltimes_container_type = typename variable_types::ltimes_variable_cont_t;
  using ltimes_variable_type  = typename variable_types::ltimes_variable_type;

  using bndry_variable_type0 =
    typename grid_boundary_traits<0, Nest>::variable_type;
  using bndry_variable_type1 =
    typename grid_boundary_traits<1, Nest>::variable_type;
  using bndry_variable_type2 =
    typename grid_boundary_traits<2, Nest>::variable_type;

  using index_traits = index_view_traits<Nest>;

  using container_type       = typename variable_types::variable_cont_t;
  using variable_type        = typename variable_types::variable_type;
  using slices_variable_type = typename variable_types::slices_variable_type;
  using sigt_slices_variable_type =
    typename variable_types::sigt_slices_variable_type;

  using index_cont_t = typename index_traits::index_cont_t;
  using index_view_t = typename index_traits::index_view_t;

  std::vector<std::vector<index_view_t *>> m_index_view;

private:
  // TODO: use std::vector<std::unique_ptr<variable_type*>> for phi,phi_out,L,L+
  std::vector<ltimes_variable_type*> m_phi;     // Moments of psi
  std::vector<ltimes_variable_type*> m_phi_out; // Scattering source (moments)
  std::vector<ltimes_variable_type*> m_ell;     // L matrix in nm_offset coords
  std::vector<ltimes_variable_type*> m_ell_plus;// L+ matrix in nm_offset coords

  // Copies of Solution and source term distributed for LTimes/LPlusTimes
  std::vector<std::vector<ltimes_variable_type*>> m_psi_ltimes;
  std::vector<std::vector<ltimes_variable_type*>> m_rhs_ltimes;

  std::vector<std::vector<variable_type*>> m_psi; // Solution
  std::vector<std::vector<variable_type*>> m_rhs; // RHS, source term

  // Zonal per-group cross-section
  std::vector<variable_type*>              m_sigt;

   // Solution
  std::vector<std::vector<slices_variable_type*>> m_sliced_psi;
  // RHS, source term
  std::vector<std::vector<slices_variable_type*>> m_sliced_rhs;

  // Zonal per-group cross-section
  std::vector<sigt_slices_variable_type*>         m_sliced_sigt;

  std::vector<std::vector<bndry_variable_type0*>> m_boundary0;
  std::vector<std::vector<bndry_variable_type1*>> m_boundary1;
  std::vector<std::vector<bndry_variable_type2*>> m_boundary2;

public:
  /*!
    \brief Build a grid from already parsed data.
  */
  Grid_Data(Input_Variables *input_vars, Directions *directions)
    : Grid_Data_Base(input_vars, directions)
  { }

  ~Grid_Data()
  {
    // TODO: change the type of m_phi, m_phi_out, m_ell, m_ell_plus to
    // std::vector<std::unique_ptr<variable_type*> > to simplify the destructor
    for (size_t i = 0; i != m_psi.size(); ++i)        // loop over groups
    {
      delete m_sliced_sigt[i];
      delete m_sigt[i];
      delete m_phi[i];
      delete m_phi_out[i];
      for (size_t j = 0; j != m_psi[i].size(); ++j)  // loop over directions
      {
        delete m_sliced_psi[i][j];
        delete m_psi[i][j];
        delete m_psi_ltimes[i][j];
        delete m_sliced_rhs[i][j];
        delete m_rhs[i][j];
        delete m_rhs_ltimes[i][j];
        delete m_boundary0[i][j];
        delete m_boundary1[i][j];
        delete m_boundary2[i][j];
        delete m_index_view[i][j];
        if (i == 0) {
          delete m_ell[j];
          delete m_ell_plus[j];
        }
        stapl::rmi_fence();
      }
    }
  }

  std::vector<ltimes_variable_type*>& phi(void)
  { return m_phi; }

  std::vector<ltimes_variable_type*> const& phi(void) const
  { return m_phi; }

  std::vector<ltimes_variable_type*>& phi_out(void)
  { return m_phi_out; }

  std::vector<ltimes_variable_type*> const& phi_out(void) const
  { return m_phi_out; }

  std::vector<ltimes_variable_type*>& ell(void)
  { return m_ell; }

  std::vector<ltimes_variable_type*> const& ell(void) const
  { return m_ell; }

  std::vector<ltimes_variable_type*>& ell_plus(void)
  { return m_ell_plus; }

  std::vector<ltimes_variable_type*> const& ell_plus(void) const
  { return m_ell_plus; }

  std::vector<std::vector<variable_type*>>& psi(void)
  { return m_psi; }

  std::vector<std::vector<variable_type*>> const& psi(void) const
  { return m_psi; }

  std::vector<std::vector<slices_variable_type*>>& sliced_psi(void)
  { return m_sliced_psi; }

  std::vector<std::vector<slices_variable_type*>> const& sliced_psi(void) const
  { return m_sliced_psi; }

  std::vector<std::vector<ltimes_variable_type*>>& psi_ltimes(void)
  { return m_psi_ltimes; }

  std::vector<std::vector<ltimes_variable_type*>> const& psi_ltimes(void) const
  { return m_psi_ltimes; }

  std::vector<std::vector<variable_type*>>& rhs(void)
  { return m_rhs; }

  std::vector<std::vector<variable_type*>> const& rhs(void) const
  { return m_rhs; }

  std::vector<std::vector<slices_variable_type*>>& sliced_rhs(void)
  { return m_sliced_rhs; }

  std::vector<std::vector<slices_variable_type*>> const& sliced_rhs(void) const
  { return m_sliced_rhs; }

  std::vector<std::vector<ltimes_variable_type*>>& rhs_ltimes(void)
  { return m_rhs_ltimes; }

  std::vector<std::vector<ltimes_variable_type*>> const& rhs_ltimes(void) const
  { return m_rhs_ltimes; }

  std::vector<variable_type*>& sigt(void)
  { return m_sigt; }

  std::vector<variable_type*> const& sigt(void) const
  { return m_sigt; }

  std::vector<sigt_slices_variable_type*>& sliced_sigt(void)
  { return m_sliced_sigt; }

  std::vector<sigt_slices_variable_type*> const& sliced_sigt(void) const
  { return m_sliced_sigt; }

  std::vector<std::vector<bndry_variable_type0*>> & boundary0(void)
  { return m_boundary0; }

  std::vector<std::vector<bndry_variable_type0*>> const& boundary0(void) const
  { return m_boundary0; }

  std::vector<std::vector<bndry_variable_type1*>> & boundary1(void)
  { return m_boundary1; }

  std::vector<std::vector<bndry_variable_type1*>> const& boundary1(void) const
  { return m_boundary1; }

  std::vector<std::vector<bndry_variable_type2*>> & boundary2(void)
  { return m_boundary2; }

  std::vector<std::vector<bndry_variable_type2*>> const& boundary2(void) const
  { return m_boundary2; }

  std::vector<std::vector<index_view_t*>> & index_view(void)
  { return m_index_view; }

  std::vector<std::vector<index_view_t*>> const& index_view(void) const
  { return m_index_view; }


  void randomizeData(void);

  bool compare(Grid_Data_Base const&, double tolerance, bool verbose);

  void copy(Grid_Data_Base const&);
};


// Work function for assigning random values to elements of inner multiarrays
struct randomize_element
{
  // value set before each map_func call to keep variables from having
  // the same values
  int unique;

  typedef void result_type;

  template<typename Multiarray>
  result_type operator()(Multiarray m)
  {
    std::tuple<int, int, int>           eidx = stapl::index_of(m);
    std::tuple<int, int, int, int, int> size = m.dimensions();

    std::mt19937_64
      gen(unique*std::get<0>(eidx)*std::get<1>(eidx)*std::get<2>(eidx));
    std::uniform_real_distribution<double> dist(0., 1.);

    std::tuple<int, int, int, int, int> idx(0,0,0,0,0);
    for (int i = 0; i != (int)std::get<0>(size); ++i){
      std::get<0>(idx) = i;
      for (int j = 0; j != (int)std::get<1>(size); ++j){
        std::get<1>(idx) = j;
        for (int k = 0; k != (int)std::get<2>(size); ++k){
          std::get<2>(idx) = k;
          for (int d = 0; d != (int)std::get<3>(size); ++d){
            std::get<3>(idx) = d;
            for (int g = 0; g != (int)std::get<4>(size); ++g){
              std::get<4>(idx) = g;
              m[idx] = dist(gen);
            }
          }
        }
      }
    }
  }

  void define_type(stapl::typer& t)
  { t.member(unique); }
};

template <Nesting_Order Nesting>
void Grid_Data<Nesting>::randomizeData(void)
{
  unsigned int loc_id = stapl::get_location_id();
  unsigned int nlocs  = stapl::get_num_locations();

  // Generator for random values
  std::mt19937_64                gen(loc_id);
  // Distribution for random values
  std::uniform_real_distribution<double> dist(0.,1.);

  for (int d = 0; d != 3; ++d)
    for (int i = 0; i != (int)m_deltas[d].size(); ++i)
      m_deltas[d][i] = dist(gen);

  randomize_element rand_wf;
  for (int gs = 0; gs != (int)m_gd_sets.size(); ++gs) {
    for (int ds = 0; ds != (int)m_gd_sets[gs].size(); ++ds) {
      rand_wf.unique += nlocs;
      stapl::map_func(rand_wf,  *m_psi[gs][ds]);
      rand_wf.unique += nlocs;
      stapl::map_func(rand_wf,  *m_rhs[gs][ds]);

      // L & L+ are stored on a per directionset basis, so they only need to
      // be processed once.
      if (gs == 0) {
        rand_wf.unique += nlocs;
        stapl::map_func(rand_wf, *m_ell[ds]);
        rand_wf.unique += nlocs;
        stapl::map_func(rand_wf, *m_ell_plus[ds]);
      }
    }
    rand_wf.unique += nlocs;
    stapl::map_func(rand_wf, *m_sigt[gs]);

    rand_wf.unique += nlocs;
    stapl::map_func(rand_wf, *m_phi[gs]);
    rand_wf.unique += nlocs;
    stapl::map_func(rand_wf, *m_phi_out[gs]);
  }
}


// work function to copy cellset multiarray
struct compare_element
{
private:
  double m_tol;
  bool   m_verbose;

public:
  std::string  name;

  typedef bool result_type;

  compare_element(double const& tol, bool verbose)
    : m_tol(tol), m_verbose(verbose)
  { }

  template<typename Multiarray, typename RefMultiarray>
  result_type operator()(Multiarray m, RefMultiarray ref)
  {
    double err       = 0.;
    int    num_wrong = 0;
    auto   size      = m.dimensions();

    std::tuple<int, int, int, int, int> idx(0,0,0,0,0);
    for (int i = 0; i != (int)std::get<0>(size); ++i){
      std::get<0>(idx) = i;
      for (int j = 0; j != (int)std::get<1>(size); ++j){
        std::get<1>(idx) = j;
        for (int k = 0; k != (int)std::get<2>(size); ++k){
          std::get<2>(idx) = k;
          for (int d = 0; d != (int)std::get<3>(size); ++d){
            std::get<3>(idx) = d;
            for (int g = 0; g != (int)std::get<4>(size); ++g){
              std::get<4>(idx) = g;

              err = std::abs(m[idx] - ref[idx]);
              if (err > m_tol) {
                if (m_verbose) {
                  printf("%s[x=%d, y=%d, z=%d, d=%d, g=%d]: |%e - %e| = %e\n",
                         name.c_str(), i, j, k, d, g,
                         (double)m[idx], (double)ref[idx], err);
                  ++num_wrong;
                  if (num_wrong > 100)
                    return true;
                }
              }
            }
          }
        }
      }
    }
    return num_wrong != 0;
  }

  void define_type(stapl::typer& t)
  {
    t.member(m_tol);
    t.member(m_verbose);
    t.member(name);
  }
};

template <Nesting_Order Nesting>
bool Grid_Data<Nesting>::compare(Grid_Data_Base const& b_base, double tol,
                                 bool verbose)
{
  Grid_Data<Nesting> const& b(static_cast<Grid_Data<Nesting> const&>(b_base));
  bool is_diff = false;
  is_diff |= compareVector("deltas[0]", m_deltas[0], b.deltas(0), tol, verbose);
  is_diff |= compareVector("deltas[1]", m_deltas[1], b.deltas(1), tol, verbose);
  is_diff |= compareVector("deltas[2]", m_deltas[2], b.deltas(2), tol, verbose);

  compare_element comp_wf(tol, verbose);
  stapl::logical_or<bool> or_wf;
  for (unsigned int gs = 0;gs < gd_sets().size();++ gs){
    for (unsigned int ds = 0;ds < gd_sets()[gs].size();++ ds){
      std::stringstream namess;
      namess << "gdset[" << gs << "][" << ds << "]";
      comp_wf.name = namess.str()+".psi";
      is_diff |= stapl::map_reduce(comp_wf, or_wf,
                   linear_view(*m_psi[gs][ds]), linear_view(*b.psi()[gs][ds]));
      comp_wf.name = namess.str()+".rhs";
      is_diff |= stapl::map_reduce(comp_wf, or_wf,
                   linear_view(*m_rhs[gs][ds]), linear_view(*b.rhs()[gs][ds]));

      // L & L+ are stored on a per directionset basis, so they only need to
      // be processed once.
      if (gs == 0) {
        namess << "dset[" << ds << "]";
        comp_wf.name = namess.str()+".ell";
        is_diff |= stapl::map_reduce(comp_wf, or_wf,
          linear_view(*m_ell[ds]), linear_view(*b.ell()[ds]));
        comp_wf.name = namess.str()+".ell_plus";
        is_diff |= stapl::map_reduce(comp_wf, or_wf,
          linear_view(*m_ell_plus[ds]), linear_view(*b.ell_plus()[ds]));
      }
    }
    std::stringstream namess;
    namess << "gset[" << gs << "]";
    comp_wf.name = namess.str()+".sigt";
    is_diff |= stapl::map_reduce(comp_wf, or_wf,
                 linear_view(*m_sigt[gs]), linear_view(*b.sigt()[gs]));

    comp_wf.name = namess.str()+".phi";
    is_diff |= stapl::map_reduce(comp_wf, or_wf,
      linear_view(*m_phi[gs]), linear_view(*b.phi()[gs]));
    comp_wf.name = namess.str()+".phi_out";
    is_diff |= stapl::map_reduce(comp_wf, or_wf,
      linear_view(*m_phi_out[gs]), linear_view(*b.phi_out()[gs]));
  }
  return is_diff;
}


// work function to copy cellset multiarray
struct copy_element
{
  typedef void result_type;

  template<typename SourceMultiarray, typename DestinationMultiarray>
  result_type operator()(SourceMultiarray source, DestinationMultiarray dest)
  {
    auto size = source.dimensions();
    std::tuple<int, int, int, int, int> idx(0,0,0,0,0);
    for (int i = 0; i != (int)std::get<0>(size); ++i){
      std::get<0>(idx) = i;
      for (int j = 0; j != (int)std::get<1>(size); ++j){
        std::get<1>(idx) = j;
        for (int k = 0; k != (int)std::get<2>(size); ++k){
          std::get<2>(idx) = k;
          for (int d = 0; d != (int)std::get<3>(size); ++d){
            std::get<3>(idx) = d;
            for (int g = 0; g != (int)std::get<4>(size); ++g){
              std::get<4>(idx) = g;
              dest[idx] = source[idx];
            }
          }
        }
      }
    }
  }
};

template <Nesting_Order Nesting>
void Grid_Data<Nesting>::copy(Grid_Data_Base const& b_base)
{
  Grid_Data<Nesting> const& b(static_cast<Grid_Data<Nesting> const&>(b_base));
  for (int d = 0; d != 3; ++d)
    this->m_deltas[d] = b.deltas(d);

  copy_element copy_wf;
  for (unsigned int gs = 0; gs != gd_sets().size(); ++gs) {
    for (unsigned int ds = 0; ds != gd_sets()[gs].size(); ++ds){
      stapl::map_func(copy_wf, *b.psi()[gs][ds], *m_psi[gs][ds]);
      stapl::map_func(copy_wf, *b.rhs()[gs][ds], *m_rhs[gs][ds]);

      // L & L+ are stored on a per directionset basis, so they only need to
      // be processed once.
      if (gs == 0) {
        stapl::map_func(copy_wf, *b.ell()[ds], *m_ell[ds]);
        stapl::map_func(copy_wf, *b.ell_plus()[ds], *m_ell_plus[ds]);
      }
    }
    stapl::map_func(copy_wf, *b.sigt()[gs], *m_sigt[gs]);

    stapl::map_func(copy_wf, *b.phi()[gs], *m_phi[gs]);
    stapl::map_func(copy_wf, *b.phi_out()[gs], *m_phi_out[gs]);
  }
}
#endif

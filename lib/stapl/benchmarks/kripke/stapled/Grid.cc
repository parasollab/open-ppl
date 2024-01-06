/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include "Grid.h"
#include "Cell.hpp"
#include "Kripke/Input_Variables.h"

#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/containers/partitions/viewbased.hpp>
#include <stapl/containers/mapping/viewbased.hpp>
#include <stapl/containers/distribution/specifications.hpp>

Grid_Data::Grid_Data(Input_Variables *input_vars, Directions *directions)
  : base_grid_type(input_vars, directions),
    px(input_vars->npx), py(input_vars->npy), pz(input_vars->npz),
    m_num_moments(input_vars->legendre_order)
{
  // My processor id and ijk
  int loc_id = this->get_location_id();

  // delinearize the location id and initialize m_mynbr.
  // We're using a zyx-major numbering of locations, similar to PDT.
  int ksub_ref = loc_id % pz;
  int jsub_ref = ((loc_id - ksub_ref) / pz) % py;
  int isub_ref = (loc_id - ksub_ref - pz*jsub_ref) / (pz * py);

  if(ksub_ref == 0)
    m_mynbr[2][0] = -1;
  else
    m_mynbr[2][0] = loc_id - 1;

  if(ksub_ref == pz-1)
    m_mynbr[2][1] = -1;
  else
    m_mynbr[2][1] = loc_id + 1;

  if(jsub_ref == 0)
    m_mynbr[1][0] = -1;
  else
    m_mynbr[1][0] = loc_id - pz;

  if(jsub_ref == py-1)
    m_mynbr[1][1] = -1;
  else
    m_mynbr[1][1] = loc_id + pz;

  if(isub_ref == 0)
    m_mynbr[0][0] = -1;
  else
    m_mynbr[0][0] = loc_id - pz * py;

  if(isub_ref == px-1)
    m_mynbr[0][1] = -1;
  else
    m_mynbr[0][1] = loc_id + pz * py;

  computeGrid(0, px, input_vars->nx, isub_ref, 0.0, 1.0);
  computeGrid(1, py, input_vars->ny, jsub_ref, 0.0, 1.0);
  computeGrid(2, pz, input_vars->nz, ksub_ref, 0.0, 1.0);

  m_num_zones = m_nzones[0] * m_nzones[1] * m_nzones[2];

  //
  // Create Zones for this processor and associated edges.
  // The property on the vertices is of type Cell3D.
  //
  using stapl::view_based_partition;
  using stapl::view_based_mapper;
  using stapl::multiarray;
  using stapl::balanced_nd;
  using stapl::dist_spec_impl::md_distribution_spec;
  using stapl::nd_linearize;

  // Create a temporary multiarray with nd block partitioning.  Use domains
  // of its base containers to drive graph vertex population.
  typedef std::tuple<std::integral_constant<int, 2>,
                std::integral_constant<int, 1>,
                std::integral_constant<int, 0>>   zyx_traversal;

  typedef md_distribution_spec<3,zyx_traversal>::type nd_partitioning_view_type;

  auto size     = std::make_tuple(input_vars->nx,input_vars->ny,input_vars->nz);

  auto loc_size =
    std::make_tuple( input_vars->npx, input_vars->npy, input_vars->npz);

  auto lin_func = nd_linearize<std::tuple<int, int, int>, zyx_traversal>(size);

  nd_partitioning_view_type nd_spec =
    stapl::balanced_nd<zyx_traversal>(size, loc_size);

  typedef view_based_partition<nd_partitioning_view_type>       part_type;
  typedef view_based_mapper<nd_partitioning_view_type>          map_type;
  typedef multiarray<3,int, zyx_traversal, part_type, map_type> vb_ma_t;

  vb_ma_t vma(nd_spec);

  auto&& cm = vma.distribution().container_manager();

  stapl_assert(cm.size() == 1, "Container Manager has more base container");

  //
  // Insert Vertices Into Graph
  //
  auto local_domain = cm.begin()->domain();
  auto local_first  = local_domain.first();
  auto local_last    = local_domain.last();

  for (auto z = std::get<2>(local_first); z <= std::get<2>(local_last); ++z)
    for (auto y = std::get<1>(local_first); y <= std::get<1>(local_last); ++y)
      for (auto x = std::get<0>(local_first); x <= std::get<0>(local_last); ++x)
        this->add_vertex(lin_func(std::make_tuple(x,y,z)), Cell3D());

  stapl::rmi_fence();

  //
  // Insert Edges Into Graph.
  //
  // The edge property currently has a normal on it indicating the direction
  // from the zone center to the face.
  //
  for (auto z = std::get<2>(local_first); z <= std::get<2>(local_last); ++z)
    for (auto y = std::get<1>(local_first); y <= std::get<1>(local_last); ++y)
      for (auto x = std::get<0>(local_first); x <= std::get<0>(local_last); ++x)
      {
        const auto cell_id = lin_func(std::make_tuple(z, y, x));

        if (x != 0)
          this->add_edge_async(
            cell_id, lin_func(std::make_tuple(x-1,y,z)),
            Vector3D(-1.0, 0.0, 0.0));

        if (x != (decltype(x))input_vars->nx-1)
          this->add_edge_async(
            cell_id, lin_func(std::make_tuple(x+1,y,z)),
            Vector3D( 1.0, 0.0, 0.0));

        if (y != 0)
          this->add_edge_async(
            cell_id, lin_func(std::make_tuple(x,y-1,z)),
            Vector3D( 0.0,-1.0, 0.0));

        if (y != (decltype(y))input_vars->ny-1)
          this->add_edge_async(
            cell_id, lin_func(std::make_tuple(x,y+1,z)),
            Vector3D( 0.0, 1.0, 0.0));

        if (z != 0)
          this->add_edge_async(
            cell_id, lin_func(std::make_tuple(x,y,z-1)),
            Vector3D( 0.0, 0.0,-1.0));

        if (z != (decltype(z))input_vars->nz-1)
          this->add_edge_async(
            cell_id, lin_func(std::make_tuple(x,y,z+1)),
            Vector3D( 0.0, 0.0, 1.0));
     }

  computeSweepIndexSets(input_vars->block_size);
} // Grid_Data::Grid_Data


void Grid_Data::computeGrid(int dim, int num_locs, int num_zones,
       stapl::location_type loc_index, double min_coord, double max_coord)
{
  /* Calculate unit roundoff and load into grid_data */
  double eps = 1e-32;
  double thsnd_eps = 1000.e0*(eps);

  // Compute subset of global zone indices
  unsigned int num_local_zones = num_zones / num_locs;
  unsigned int rem = num_zones % num_locs;
  unsigned int ilower, iupper;
  if (rem != 0) {
    if (loc_index < rem){
      ++num_local_zones;
      ilower = loc_index * num_local_zones;
    }
    else
      ilower = rem + loc_index * num_local_zones;
  }
  else
  {
    ilower = loc_index * num_local_zones;
  }

  iupper = ilower + num_local_zones - 1;

  // allocate grid deltas
  m_deltas[dim].resize(num_local_zones+2);

  // Compute the spatial grid
  double dx = (max_coord - min_coord) / num_zones;
  double coord_lo = min_coord + (ilower) * dx;
  double coord_hi = min_coord + (iupper+1) * dx;
  for(unsigned int i = 0; i < num_local_zones+2; i++){
    m_deltas[dim][i] = dx;
  }
  if(std::abs(coord_lo - min_coord) <= thsnd_eps*std::abs(min_coord)){
    m_deltas[dim][0] = 0.0;
  }
  if(std::abs(coord_hi - max_coord) <= thsnd_eps*std::abs(max_coord)){
    m_deltas[dim][num_local_zones+1] = 0.0;
  }

  m_nzones[dim] = num_local_zones;
}


struct equal_properties
{
  typedef bool result_type;

  template <typename Vertex>
  result_type operator()(Vertex v1, Vertex v2)
  { return v1.property() == v2.property(); }
};

bool Grid_Data::compare(Grid_Data const& b, double tol, bool verbose)
{
  bool is_diff = false;
  is_diff |= compareVector("deltas[0]", m_deltas[0], b.deltas(0), tol, verbose);
  is_diff |= compareVector("deltas[1]", m_deltas[1], b.deltas(1), tol, verbose);
  is_diff |= compareVector("deltas[2]", m_deltas[2], b.deltas(2), tol, verbose);

  for(unsigned int gs = 0;gs < gd_sets().size();++ gs){
    for(unsigned int ds = 0;ds < gd_sets()[gs].size();++ ds){
      is_diff |= gd_sets()[gs][ds].compare(
        gs, ds, b.gd_sets()[gs][ds], tol, verbose);
    }
  }

  stapl::graph_view<Grid_Data> tv(*this);
  stapl::graph_view<Grid_Data> bv(b);
  is_diff |= stapl::map_reduce(equal_properties(), stapl::logical_and<bool>(),
               tv, bv);

  return is_diff;
}


struct copy_properties
{
  typedef void result_type;

  template <typename Vertex>
  result_type operator()(Vertex v1, Vertex v2)
  { return v1.property().copy(v2.property()); }
};

void Grid_Data::copy(Grid_Data const& b)
{
  for (int d = 0; d != 3; ++d)
    m_deltas[d] = b.deltas(d);

  for (unsigned int gs = 0; gs != gd_sets().size(); ++gs)
    for (unsigned int ds = 0; ds != gd_sets()[gs].size(); ++ds)
      gd_sets()[gs][ds].copy(b.gd_sets()[gs][ds]);

  stapl::graph_view<Grid_Data> tv(*this);
  stapl::graph_view<Grid_Data> bv(b);
  stapl::map_func(copy_properties(), tv, bv);
}


/**
 * Computes index sets for each octant, and each tile (experimental).
 * Determines logical indices, and increments for i,j,k based on grid
 * information and quadrature set sweeping direction.
 */
void Grid_Data::computeSweepIndexSets(int block_size){
  m_octant_extent.resize(8);
  for(int octant = 0;octant < 8;++ octant){

    int id, jd, kd;
    switch(octant){
      case 0: id = 1; jd = 1; kd = 1; break;
      case 1: id = -1; jd = 1; kd = 1; break;
      case 2: id = -1; jd = -1; kd = 1; break;
      case 3: id = 1; jd = -1; kd = 1; break;
      case 4: id = 1; jd = 1; kd = -1; break;
      case 5: id = -1; jd = 1; kd = -1; break;
      case 6: id = -1; jd = -1; kd = -1; break;
      case 7: id = 1; jd = -1; kd = -1; break;
    }

    int istartz, istopz, in;//, il, ir;

    if(id > 0){
      istartz = 0; istopz = m_nzones[0]-1; in = 1;// il = 0; ir = 1;
    }
    else {
      istartz = m_nzones[0]-1; istopz = 0; in = -1;// il = 1; ir = 0;
    }

    int jstartz, jstopz, jn;//, jf, jb;
    if(jd > 0){
      jstartz = 0; jstopz = m_nzones[1]-1; jn = 1;// jf = 0; jb = 1;
    }
    else {
      jstartz = m_nzones[1]-1; jstopz = 0; jn = -1;// jf = 1; jb = 0;
    }

    int kstartz, kstopz, kn;//, kb, kt;
    if(kd > 0){
      kstartz = 0; kstopz = m_nzones[2]-1; kn =  1;// kb = 0; kt = 1;
    }
    else {
      kstartz = m_nzones[2]-1; kstopz = 0; kn = -1;// kb = 1; kt = 0;
    }

    // Define extent block and pattern
    Grid_Sweep_Block &extent = m_octant_extent[octant];
    extent.start_i = istartz;
    extent.start_j = jstartz;
    extent.start_k = kstartz;
    extent.end_i = istopz + in;
    extent.end_j = jstopz + jn;
    extent.end_k = kstopz + kn;
    extent.inc_i = in;
    extent.inc_j = jn;
    extent.inc_k = kn;
  }
}

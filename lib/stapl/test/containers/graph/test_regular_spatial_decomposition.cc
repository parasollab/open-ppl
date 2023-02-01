/*
// Copyright (c) 2000-2011, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/mesh/implicit_regular_mesh.hpp>
#include <stapl/containers/graph/partitioners/graph_partitioners.h>
#include <stapl/containers/graph/partitioners/regular_spatial_decomposition.hpp>
#include <test/test_report.hpp>

using namespace stapl;

template<int N>
bool test_regular_spatial_decomposition
(const typename implicit_regular_mesh_container<N>::tuple_type& size,
 const typename implicit_regular_mesh_container<N>::tuple_type& decomposition)
{
  //Constructing implicit regular mesh
  typedef typename implicit_regular_mesh_view_type<N>::type mesh_view_type;
  mesh_view_type mesh_vw = make_implicit_regular_mesh_view<N>(size);

  //Partitioning
  gpartition<mesh_view_type> gp = graph_partition(mesh_vw, spatial_decomposition<N>(decomposition));

  //Check partition coherency
  size_t num_partitions   = tuple_ops::fold(decomposition, 1, multiplies<size_t>());
  size_t res_num_vertices = 0;

  for(size_t i=0; i<gp.size(); ++i)
  {
    res_num_vertices += gp.vertex_size(i);
  }

  bool res = (gp.size() == num_partitions) && (res_num_vertices == mesh_vw.size());

  rmi_fence();

  return res;
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc!=6)
  {
    std::cout << "Format: " << argv[0] << " num_cells_x num_cells_y num_cells_z dec_x dec_y" << std::endl;
    return EXIT_FAILURE;
  }

  size_t a1 = atoi(argv[1]);
  size_t a2 = atoi(argv[2]);
  size_t a3 = atoi(argv[3]);
  size_t a4 = atoi(argv[4]);
  size_t a5 = atoi(argv[5]);

  //KBA 2D
  bool kba_2d_1 = test_regular_spatial_decomposition<2>(make_tuple(a1, a2), KBA<2>());
  bool kba_2d_2 = test_regular_spatial_decomposition<2>(make_tuple(a1, a2), KBA(a4));

  //KBA 3D
  bool kba_3d_1 = test_regular_spatial_decomposition<3>(make_tuple(a1, a2, a3), KBA<3>());
  bool kba_3d_2 = test_regular_spatial_decomposition<3>(make_tuple(a1, a2, a3), KBA(a4, a5));

  bool res = kba_2d_1 && kba_2d_2 && kba_3d_1 && kba_3d_2;

  STAPL_TEST_REPORT(res,"Testing regular spatial decomposition\t");

  return EXIT_SUCCESS;
}

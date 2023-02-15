/*
// Copyright (c) 2000-2011, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <test/test_report.hpp>
#include <stapl/containers/graph/mesh/unstructured_mesh.hpp>
#include <stapl/containers/graph/mesh/generate_output_mesh_file.hpp>
#include <stapl/containers/graph/mesh/import_mesh_file.hpp>
#include <stapl/containers/unordered_map/unordered_map.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/partitioners/arbitrary_mesh_decomposition.hpp>
#include <stapl/containers/graph/partitioners/graph_partitioners.h>
#include <stapl/containers/graph/partitioners/graph_partitioner_utils.hpp>
#include <stapl/containers/graph/partitioners/gpartition.h>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <string>

using namespace stapl;
using namespace std;

struct get_partition_id
{
  typedef size_t value_type;
  template<typename Property>
  value_type get(Property p)
  {
    return p.get_partition_id();
  }
};

stapl::exit_code stapl_main(int argc, char** argv)
{
  if (argc < 5) {
    std::cerr << "usage: " << argv[0] << " filename3D num_partitions_x"
    " num_partitions_y num_partitions_z" << std::endl;
    return EXIT_FAILURE;
  }

  typedef unstructured_mesh<3> mesh_type;
  typedef geom_vector<3, double> geom_vector_type;
  typedef graph_view<mesh_type> view_type;
  string filename(argv[1]);
  size_t num_x = atoi(argv[2]);
  size_t num_y = atoi(argv[3]);
  size_t num_z = atoi(argv[4]);
  size_t num_part = num_x*num_y*num_z;

  mesh_type mesh = mesh_type(import_silo_file(filename));
  view_type mesh_vw(mesh);

  gpartition<view_type>
  gp = graph_partition(mesh_vw, arbitrary_mesh_collapser<3, 3, double>(
                       make_tuple(num_x, num_y, num_z),
                       make_tuple(geom_vector_type(0, 1, 0),
                       geom_vector_type(1, 0, 0), geom_vector_type(0, 0, 1))));


  typedef stapl::graph_internal_property_map<mesh_type, get_partition_id>
  p_property_map_view_type;
  //Just for test, this map does not store any value.
  p_property_map_view_type partition_id_map(mesh, get_partition_id());

  //output mesh in a silo file
  generate_silo_file(mesh, partition_id_map, "part_mesh_3d");

  //Check partition coherency
  size_t res_num_vertices=0;
  for (size_t i=0;i<gp.size();++i)
  {
    res_num_vertices+=gp.vertex_size(i);
  }

  bool empty_intersection=true;
  for (size_t i=0;i<gp.size();++i)
  {
    for (size_t j=0;j<gp.size();++j)
    {
      if (i != j)
        empty_intersection =
          empty_intersection && (gp.vertices(i) & gp.vertices(j)).empty();
    }
  }

  bool res = (gp.size()==num_part) && (res_num_vertices==mesh_vw.size())
                                   && empty_intersection;
  STAPL_TEST_REPORT(res,"Testing 3D unstructured mesh partitioner\t");

  return EXIT_SUCCESS;
}

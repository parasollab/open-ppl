/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <test/test_report.hpp>
#include <stapl/containers/graph/partitioners/gpartition.h>
#include <stapl/containers/graph/partitioners/graph_partitioners.h>
#include <stapl/containers/graph/partitioners/graph_partitioner_utils.hpp>
#include <stapl/containers/graph/partitioners/multilevel.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/digraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/generators/torus.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>

using namespace stapl;

//////////////////////////////////////////////////////////////////////
/// @brief Functor to access and set a weight on a graph property.
//////////////////////////////////////////////////////////////////////
class access_weight
{
public:
  typedef size_t value_type;

  //////////////////////////////////////////////////////////////////////
  /// @brief Returns weight of a property.
  /// @param vp property reference.
  /// @return property weight.
  //////////////////////////////////////////////////////////////////////
  template <typename VProperty>
  value_type get(VProperty const& vp)
  {
    return vp;
  }

  //////////////////////////////////////////////////////////////////////
  /// @brief Assign a weight to a property.
  /// @param vp property reference.
  /// @param weight weight assigned.
  //////////////////////////////////////////////////////////////////////
  template <typename VProperty>
  void put(VProperty vp, value_type const& weight)
  {
   vp = weight;
  }
};

//////////////////////////////////////////////////////////////////////
/// @brief Set weights of a vertex and its edges to 1.
//////////////////////////////////////////////////////////////////////
struct init_property
{
  typedef void result_type;

  //////////////////////////////////////////////////////////////////////
  /// @param v vertex proxy.
  //////////////////////////////////////////////////////////////////////
  template <typename Vertex>
  void operator()(Vertex v) const
  {
    v.property() = 1;
    typename Vertex::adj_edge_iterator it = v.begin(),
                                    end_it = v.end();
    for (; it != end_it; ++it)
      (*it).property()=1;
  }
};


stapl::exit_code stapl_main(int argc, char** argv)
{
  typedef digraph<size_t, size_t> pgraph_type;
  typedef pgraph_type::vertex_property vertex_property;
  typedef graph_view<pgraph_type> pgraph_view;
  typedef gpartition<pgraph_view> gpartition_type;

  if (argc!=7)
  {
    std::cout << "Format: " << argv[0] << " number_partitions imbalance_factor"
              << " coarsening_factor number_refinement_passes"
              << " torus_num_cells_x torus_num_cells_y" << std::endl;
    return EXIT_FAILURE;
  }

  size_t num_part = atol(argv[1]);
  size_t imbalance = atol(argv[2]);
  size_t coarsening_factor = atol(argv[3]);
  size_t fm_pass = atol(argv[4]);
  size_t nx = atol(argv[5]);
  size_t ny = atol(argv[6]);

  pgraph_view vw = stapl::generators::make_torus<pgraph_view>(nx, ny, true);

  map_func(init_property(), vw);

  typedef graph_internal_property_map<pgraph_view,
                                      access_weight> vertex_map_type;
  typedef graph_edge_property_map<pgraph_view,access_weight> edge_map_type;
  vertex_map_type weight_map(vw);
  edge_map_type edge_weight_map(vw);

  gpartition_type gp =
    graph_partition(vw, k_way_multilevel<vertex_map_type, edge_map_type>
                          (weight_map, edge_weight_map, num_part, imbalance,
                           coarsening_factor, fm_pass));

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

  bool res = (gp.size()==num_part) && (res_num_vertices==vw.size())
                                   && empty_intersection;
  STAPL_TEST_REPORT(res,"Testing parallel k-way multilevel partitioner\t");

  return EXIT_SUCCESS;
}

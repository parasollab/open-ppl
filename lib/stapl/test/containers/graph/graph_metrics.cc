/*
 // Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
 // component of the Texas A&M University System.

 // All rights reserved.

 // The information and source code contained herein is the exclusive
 // property of TEES and may not be disclosed, examined or reproduced
 // in whole or in part without explicit written authorization from TEES.
 */

#include <stapl/containers/graph/multidigraph.hpp>
#include <stapl/containers/graph/algorithms/graph_metrics.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/containers/graph/generators/mesh.hpp>
#include <stapl/containers/graph/generators/torus.hpp>
#include "test_util.h"


void verify_mesh(size_t nx, size_t ny, size_t p, stapl::metrics_info& i)
{
  ny = ny / p;
  one_print(i.sum_internal_edges == ((2 * nx * ny - nx - ny) * p) &&
            i.sum_cross_edges == (nx * (p - 1)));
}


void verify_torus(size_t nx, size_t ny, size_t p, stapl::metrics_info& i)
{
  ny = ny / p;
  if (p == 1) {
    one_print(i.sum_internal_edges == (2 * nx * ny) && i.sum_cross_edges == 0);
  } else {
    one_print(i.sum_internal_edges == ((2 * nx * ny - nx) * p)
              && i.sum_cross_edges == nx * p);
  }
}


class init_wf
{
private:
  stapl::location_type m_id;

public:
  init_wf(stapl::location_type id)
    : m_id(id)
  { }

  typedef void result_type;

  template<typename V, typename PropMap>
  void operator()(V v, PropMap p) const
  {
    p.put(v, m_id);
  }

  void define_type(stapl::typer& t)
  {
    stapl::abort("init_wf serialization unexepectedly attempted");
  }
};


struct identity_func
{
  typedef size_t value_type;

  template<typename Property>
  value_type get(Property p) const
  { return p; }

  template<typename Property>
  void put(Property p, value_type v) const
  { p = v; }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t nx, ny;
  if (argc == 3) {
    nx = atol(argv[1]);
    ny = atol(argv[2]);
  } else {
    std::cout << "Usage: exe nx ny;\nUsing 10x4 by default\n";
    nx = 10;
    ny = 4;
  }

  ny = ny * stapl::get_num_locations();

  typedef stapl::multidigraph<size_t> PGR;
  typedef stapl::graph_view<PGR> graph_vw_t;
  typedef stapl::graph_internal_property_map<graph_vw_t, identity_func>
    vertex_part_map_t;

  one_print("Testing graph metrics for mesh...\t");
  graph_vw_t mesh_vw = stapl::generators::make_mesh<graph_vw_t>(nx, ny, false);

  vertex_part_map_t vertex_part_map1(mesh_vw, identity_func());

  stapl::map_func(init_wf(stapl::get_location_id()),
                  mesh_vw, make_repeat_view(vertex_part_map1));

  stapl::metrics_info resultm =
    graph_metrics(mesh_vw.container(), vertex_part_map1);

  verify_mesh(nx, ny, stapl::get_num_locations(), resultm);

  stapl::rmi_fence();

  one_print("Testing graph metrics for torus...\t");
  graph_vw_t torus_vw =
    stapl::generators::make_torus<graph_vw_t>(nx, ny, false);

  vertex_part_map_t vertex_part_map2(torus_vw, identity_func());
  stapl::map_func(init_wf(stapl::get_location_id()),
                  torus_vw, make_repeat_view(vertex_part_map2));

  stapl::metrics_info resultt = graph_metrics(torus_vw.container(),
                                              vertex_part_map2);
  verify_torus(nx, ny, stapl::get_num_locations(), resultt);

  return EXIT_SUCCESS;
}

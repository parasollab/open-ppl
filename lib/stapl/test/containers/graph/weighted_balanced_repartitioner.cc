/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/paragraph/paragraph.hpp>
#include <stapl/containers/graph/partitioners/gpartition.h>
#include <stapl/containers/graph/partitioners/refiners.hpp>
#include <stapl/containers/graph/partitioners/graph_partitioners.h>
#include <stapl/containers/graph/partitioners/graph_partitioner_utils.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/algorithms/hierarchical_view.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <test/test_report.hpp>
#include <time.h>

#include "test_util.h"

using namespace std;
using namespace stapl;

struct get_weight_property
{
  typedef size_t value_type;
  template<typename Property>
  value_type get(Property p)
  {
    return p;
  }
};

struct wf_random
{
  typedef void result_type;

  wf_random(size_t const& max_w) : max_weight(max_w)
  {
    srand(time(NULL)+stapl::get_location_id());
  }

  template<typename Elt>
  void operator()(Elt elt)
  {
    elt.property()=rand()%max_weight+1;
  }

  void define_type(stapl::typer& t)
  {
    t.member(max_weight);
  }

private:
  size_t max_weight;
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  typedef stapl::dynamic_graph<
            stapl::DIRECTED,stapl::MULTIEDGES, size_t> pgraph_type;
  typedef stapl::graph_view<pgraph_type> pgraph_view;
  typedef stapl::gpartition<pgraph_view> gpartition_t;
  pgraph_type pg;
  pgraph_view vw(pg);

  size_t myid=stapl::get_location_id();
  size_t num_locs=stapl::get_num_locations();

  typedef stapl::graph_internal_property_map<
            pgraph_view, get_weight_property> p_property_map_view_type;
  p_property_map_view_type weight_map(vw, get_weight_property());

  if (argc!=3)
  {
    cout << "Format: " << argv[0]
         << " <Max number of vertices per location> <Max vertex weight value> "
         << endl;
    return EXIT_FAILURE;
  }

  //Initialize random number generator
  srand ( myid+time(NULL) );

  int max_num_vertices = atoi(argv[1]);
  if (max_num_vertices == 0)
  {
    cout << " Max number of vertices per location must be greater than 0.\n";
    return EXIT_FAILURE;
  }
  size_t num_vertices=rand()%max_num_vertices+1;
  for (size_t i=0;i<num_vertices;++i)
  {
    vw.add_vertex();
  }

  stapl::rmi_fence();

  size_t total_vertices=vw.size();

  //Add random weights to the vertices
  stapl::map_func(wf_random(atoi(argv[2])), vw);

  //Getting the native partitions
  gpartition_t native_gp(stapl::create_level(
                           vw,
                           stapl::native_partitioner<pgraph_view>(),
                           stapl::native_ef()) );

#ifdef DEBUG_INFO
  if (myid == 0){
    cout << "native gpartition_size: " << native_gp.size() << endl;
    for (size_t i=0;i<native_gp.size();++i)
    {
      cout << "domain " << i << ": " << native_gp.vertices(i) << endl;
    }
  }
#endif

  //Layout of the mesh of partitions
  size_t num_x, num_y;
  size_t sqrt_num = sqrt(num_locs);
  if (sqrt_num*sqrt_num == num_locs)
  {
    num_x=num_y=sqrt_num;
  }
  else if (num_locs%2)
  {
   num_x=num_locs;
   num_y=1;
  }
  else
  {
   num_x=num_locs/2;
   num_y=2;
  }

 //Repartition the graph
  gpartition_t gp=stapl::graph_repartition( native_gp,
  stapl::weight_balanced_refiner<p_property_map_view_type>(
    weight_map, mesh_independent_sets(num_x, num_y), 0) );

#ifdef DEBUG_INFO
  if (myid == 0)
  {
    cout << "weight-balanced gpartition_size: " << gp.size() << endl;
    for (size_t i=0;i<gp.size();++i)
    {
      cout << "domain " << i << ": " << gp.vertices(i) << endl;
    }
  }
#endif

  //Check partition coherency//

  //Check number of vertices in partitions
  size_t res_num_vertices=0;
  for (size_t i=0;i<gp.size();++i)
  {
    res_num_vertices+=gp.vertex_size(i);
  }

  //Check if vertex uniqueness across partitions
  bool empty_intersection=true;
  for (size_t i=0;i<gp.size();++i)
  {
    for (size_t j=i+1;j<gp.size();++j)
    {
      empty_intersection= empty_intersection &&
                          (gp.vertices(i)&gp.vertices(j)).empty();
    }
  }

  bool res = (gp.size()==num_locs) &&
             (res_num_vertices==total_vertices) &&
             empty_intersection;
  STAPL_TEST_REPORT(res,"Testing weighted_balanced_repartitioner\t");

  return EXIT_SUCCESS;
}

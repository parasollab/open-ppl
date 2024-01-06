/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

/*!
  \file hierarchical_view.cc
  \ingroup stapl
  \brief Test suite for pGraph Hierarchical View.
*/

#include <iostream>
#include <utility>

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/hierarchical_view.hpp>
#include <stapl/containers/graph/generators/torus.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "../../test_report.hpp"

using namespace stapl;
using namespace std;


struct wf_h4
{
  typedef size_t result_type;

  template <class T1>
  result_type operator()(T1 v1) const
  {
    return v1.descriptor();
  }
};


struct wf_h3
{
  typedef size_t result_type;

  template <class T1>
  result_type operator()(T1 v1) const
  {
    typename T1::property_type p = v1.property();
    return stapl::map_reduce(wf_h4(), stapl::plus<size_t>(), p);
  }
};


struct wf_h2
{
  typedef size_t result_type;

  template <class T1>
  result_type operator()(T1 v1) const
  {
    typename T1::property_type p = v1.property();
    return stapl::map_reduce(wf_h3(), stapl::plus<size_t>(), p);
  }
};


struct EF
{
  typedef int value_type;

  template<class Graph>
  void operator()(Graph* g, size_t lvl) const
  {
    size_t blk_sz = g->num_vertices()/get_num_locations();
    size_t blk_bg = blk_sz*get_location_id();

    typename Graph::vertex_iterator vi = g->begin(), vi_end = g->end(), vi_temp;
    for (size_t i=0; i<blk_bg; ++i)
    { ++vi; }
    vi_temp = vi; ++vi_temp;

    for (size_t i=blk_bg; i<blk_bg+blk_sz; ++i)
    {
      if (vi_temp == vi_end)
        vi_temp = g->begin();
      g->add_edge_async((*vi).descriptor(), (*vi_temp).descriptor());
      ++vi, ++vi_temp;
    }
  }
};


struct HPartitioner
{
  typedef domset1D<size_t>                        setdom_type;
  typedef stapl::array<size_t>                    descriptor_container_type;
  typedef stapl::array<setdom_type>               set_container_type;
  typedef array_view<set_container_type>          view_type;
  typedef array_view<descriptor_container_type>   descriptor_view_type;

  template<class GV>
  std::pair<view_type, descriptor_view_type>
  operator()(GV const& g, size_t lvl) const
  {
    size_t id = get_location_id();
    size_t num_lcl_parts = pow(2, 4-lvl);
    size_t num_parts = num_lcl_parts*get_num_locations();
    size_t lcl_start = num_lcl_parts*id;

    set_container_type* init_vec = new set_container_type(num_parts);
    descriptor_container_type* descriptor_vec =
      new descriptor_container_type(num_parts);

    // create explicit domains independantly on each location
    // -- must fence after this to synchronize!
    for (size_t i=lcl_start; i<lcl_start+num_lcl_parts; ++i)
    {
      setdom_type v;
      for (size_t j=i; j<g.size(); j+=num_parts)
      {
        v += j;
      }
      init_vec->operator[](i) = v;
      descriptor_vec->operator[](i) = i;
    }
    rmi_fence();

    // create partitions; must fence after this to synchronize!
    descriptor_view_type dv(descriptor_vec);
    rmi_fence();

    return make_pair(view_type(init_vec), dv);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t nx,ny;
  if (argc > 2) {
    nx = atol(argv[1]);
    ny = atol(argv[2]);
  } else {
    cout<<"Usage: exe x y; Using 10x4 torus by default\n";
    nx=10;ny=4;
  }

  nx = nx * get_num_locations();
  size_t nv = nx*ny;
  srand(nx);

  typedef dynamic_graph<DIRECTED, MULTIEDGES, int, int> PGR;
  typedef graph_view<PGR> graph_vw_t;

  graph_vw_t gvw = generators::make_torus<graph_vw_t>(nx, ny, false);

  size_t sum_of_descriptors =
    map_reduce(wf_h2(), stapl::plus<size_t>(),
               create_level(
                 create_level(gvw, HPartitioner(), EF()),
                   HPartitioner(), EF())
               );

  STAPL_TEST_REPORT(sum_of_descriptors == nv*(nv-1)/2,
                    "Testing hierarchical_views");

  return EXIT_SUCCESS;
}

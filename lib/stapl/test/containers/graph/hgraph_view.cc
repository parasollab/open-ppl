/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <iostream>
#include <utility>

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/containers/graph/hierarchical_graph.hpp>
#include <stapl/containers/graph/views/hgraph_view.hpp>
#include <stapl/containers/graph/generators/torus.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/domains/interval.hpp>

#include "../../test_report.hpp"

using namespace stapl;
using namespace std;


struct wf_h3
{
  typedef size_t result_type;

  template <class T1>
  result_type operator()(T1 v1) const
  {
    return v1.descriptor();
  }
};


template<typename GView>
struct wf_h2
{
  GView const*   graph;
  typedef size_t result_type;

  wf_h2(GView const& vw)
    : graph(&vw)
  { }

  template <class T1>
  result_type operator()(T1 v1) const
  {
    typename T1::vertex_descriptor vid = v1.descriptor();
    return stapl::map_reduce(wf_h3(), stapl::plus<size_t>(),
                             graph->children(vid));
  }

  void define_type(typer &t)
  {
    t.member(graph);
  }
};


template<typename GView>
struct wf_h1
{
  GView const* graph;

  typedef size_t result_type;

  wf_h1(GView const& vw)
    : graph(&vw)
  { }

  template <class T1>
  result_type operator()(T1 v1) const
  {
    const typename T1::vertex_descriptor vid = v1.descriptor();
    return stapl::map_reduce(
      wf_h2<GView>(*graph), stapl::plus<size_t>(), graph->children(vid)
    );
  }

  void define_type(typer &t)
  {
    t.member(graph);
  }
};


struct EF
{
  typedef int value_type;

  template<class Graph>
  void operator()(Graph& g, size_t lvl) const
  {
    const size_t blk_sz = g.size() / get_num_locations();
    const size_t blk_bg = blk_sz   * get_location_id();

    typedef typename Graph::edge_descriptor edge_descriptor;

    typename Graph::vertex_iterator vi     = g.begin();
    typename Graph::vertex_iterator vi_end = g.end();
    typename Graph::vertex_iterator vi_temp;

    for (size_t i=0; i<blk_bg; ++i)
    { ++vi; }

    vi_temp = vi;
    ++vi_temp;

    for (size_t i = blk_bg; i < blk_bg + blk_sz; ++i)
    {
      if (vi_temp == vi_end)
        vi_temp = g.begin();
      g.add_edge_async(edge_descriptor((*vi).descriptor(),
                                       (*vi_temp).descriptor()));
      ++vi, ++vi_temp;
    }

    rmi_fence();
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
  operator()(GV& g, size_t lvl) const
  {
    size_t id            = get_location_id();
    size_t num_lcl_parts = pow(2, 4-lvl);
    size_t num_parts     = num_lcl_parts*get_num_locations();
    size_t lcl_start     = num_lcl_parts*id;
    size_t offset        = compute_level_offset::default_offset;

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
        v += (lvl-1)*offset + j;
        // printf("%d: adding %d to domain %d.\n", (int)id, (int)j, (int)i);
      }

      init_vec->operator[](i)       = v;
      descriptor_vec->operator[](i) = i;
    }

    // create partitions; must fence after this to synchronize!
    descriptor_view_type dv(descriptor_vec);
    rmi_fence();

    return make_pair(view_type(init_vec), dv);
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t nx;
  size_t ny;

  if (argc > 2)
  {
    nx = atol(argv[1]);
    ny = atol(argv[2]);
  }
  else
  {
    cout << "Usage: exe x y; Using 10x4 torus by default\n";
    nx = 10;
    ny = 4;
  }

  typedef hierarchical_graph<DIRECTED, MULTIEDGES, int, int> PGR;

  nx = nx * get_num_locations();

  srand(nx);
  PGR p(nx*ny);

  typedef graph_view<PGR> graph_vw_t;
  graph_vw_t gvw(p);

  gvw = generators::make_torus<graph_vw_t>(gvw, nx, ny, false);

  hgraph_view<PGR> fullview(p, true);
  hgraph_view<PGR> hview0(p);

  const size_t sum_of_descriptors = map_reduce(
    wf_h1<hgraph_view<PGR> >(fullview),
    stapl::plus<size_t>(),
    create_hierarchy(
      create_hierarchy(hview0, HPartitioner(), EF()),
      HPartitioner(), EF()
    )
  );

  STAPL_TEST_REPORT(sum_of_descriptors == hview0.size()*(hview0.size()-1)/2,
                    "Testing hierarchical_graph_views");

  return EXIT_SUCCESS;
}

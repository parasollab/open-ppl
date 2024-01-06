/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <stapl/runtime.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/views/property_maps.hpp>
#include <stapl/containers/graph/algorithms/rebalance_global.hpp>
#include <stapl/containers/partitions/normal.hpp>

#include <stapl/views/counting_view.hpp>
#include <stapl/algorithms/algorithm.hpp>

#include "test_util.h"

struct size_wf
{
  typedef size_t result_type;

  template <typename NatVw>
  result_type operator()(NatVw ng)
  {
    return ng.size();
  }
};


struct wt_wf
{
  typedef size_t result_type;

  template <typename NatVw>
  result_type operator()(NatVw ng)
  {
    size_t total_wt = 0;
    for (typename NatVw::vertex_iterator i=ng.begin(); i!=ng.end(); ++i)
      total_wt += (*i).property();
    return total_wt;
  }
};


struct id_f
{
  typedef int value_type;

  template<typename P>
  value_type get(P p) const
  { return p; }
};


template<typename Vw, typename Wf>
typename Wf::result_type max_property(Vw v, Wf wf)
{
  return stapl::map_reduce(wf, stapl::max<typename Wf::result_type>(), v);
}


template<typename Vw, typename Wf>
typename Wf::result_type min_property(Vw v, Wf wf)
{
  return stapl::map_reduce(wf, stapl::min<typename Wf::result_type>(), v);
}


struct assign_unique_weights
{
  template<typename Vertex>
  void operator()(Vertex v)
  {
    v.property() = v.descriptor();
  }
};


stapl::exit_code stapl_main(int argc, char* argv[])
{
  if (argc < 3) {
    std::cerr << "usage: exe n ef" << std::endl;
    exit(1);
  }
  srand(1000*stapl::get_location_id() + time(NULL));
  size_t n  = atoi(argv[1]);
  size_t ef = atoi(argv[2]);

  typedef stapl::indexed_domain<size_t>                    domain_type;
  typedef stapl::normal_partition<domain_type>             partition_type;
  typedef stapl::dynamic_graph<stapl::DIRECTED,
            stapl::MULTIEDGES, int, stapl::properties::no_property,
            partition_type>                                graph_type;

  domain_type dom(0, n-1);
  partition_type part(dom, stapl::get_num_locations(), 0, 1.0);

  graph_type g(part);

  for (size_t i=0; i<ef; ++i) {
    size_t s = rand()%(n-1);
    size_t t = rand()%(n-1);
    g.add_edge_async(s, t);
  }
  stapl::rmi_fence();

  typedef stapl::graph_view<graph_type> graph_view_type;
  graph_view_type gvw(g);

  double local_elts_max_initial = max_property(stapl::native_view(gvw),
                                               size_wf());
  double local_elts_min_initial = min_property(stapl::native_view(gvw),
                                               size_wf());

  one_print("Testing Global Rebalance (Non-Weighted)...");

  // rebalance without considering vertex weights:
  stapl::rebalance_global(gvw);

  // rebalance invalidates the view, so we need to reconstruct it before
  // further usage
  gvw = graph_view_type(g);

  double local_elts_max_rbnw = max_property(stapl::native_view(gvw), size_wf());
  double local_elts_min_rbnw = min_property(stapl::native_view(gvw), size_wf());

  one_print(local_elts_max_rbnw < local_elts_max_initial &&
            local_elts_min_rbnw > local_elts_min_initial);


  one_print("Testing Global Rebalance (Weighted)...\t");

  // create unique weights
  stapl::graph_internal_property_map<graph_view_type, id_f> weights(gvw,
                                                                    id_f());
  stapl::graph_view<graph_type> gvw2(g);

  stapl::for_each(gvw2, assign_unique_weights());

  double local_wt_max_initial = max_property(stapl::native_view(gvw2), wt_wf());
  double local_wt_min_initial = min_property(stapl::native_view(gvw2), wt_wf());

  // rebalance considering vertex weights:
  stapl::rebalance_global(gvw2, weights);

  // rebalance invalidates the view, so we need to reconstruct it before
  // further usage
  gvw2 = graph_view_type(g);

  double local_wt_max_rbw = max_property(stapl::native_view(gvw2), wt_wf());
  double local_wt_min_rbw = min_property(stapl::native_view(gvw2), wt_wf());

  one_print(local_wt_max_rbw < local_wt_max_initial &&
            local_wt_min_rbw > local_wt_min_initial);

  return EXIT_SUCCESS;
}

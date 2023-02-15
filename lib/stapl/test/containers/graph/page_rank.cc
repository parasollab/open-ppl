/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

/**
 *  This file is for testing PageRank for usage in RMAT benchmarking/correctness
 */


//STAPL includes:
#include <stapl/containers/graph/multigraph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/containers/graph/algorithms/page_rank.hpp>
#include <stapl/containers/graph/algorithms/create_level_machine.hpp>
#include <stapl/containers/graph/algorithms/create_level_hubs.hpp>
#include <stapl/containers/graph/algorithms/properties.hpp>
#include <stapl/containers/graph/algorithms/aggregator.hpp>
#include <stapl/containers/graph/generators/torus.hpp>
#include <stapl/algorithms/algorithm.hpp>
// #include "graph500_generator/make_graph.h"

#include "test_util.h"

using namespace stapl;
using namespace std;

size_t NEXP  = 1;
size_t SCALE = 12;
size_t SITER = 20;
size_t MAX_AGGR_MSG_SZ = 16*1024;
size_t MAX_EDGE_AGGR_MSG_SZ = 16*1024;


struct vertex_extract_wf
{
  typedef double result_type;
  template<typename V>
  result_type operator() (V v)
  {
    return v.property().rank();
  }
};


template<typename G>
bool check_result(G& g)
{
  double res = map_reduce(vertex_extract_wf(), stapl::plus<double>(), g);
  if (abs(res - g.num_vertices()) < 0.01)
    return true;
  else return false;
}

template<typename G>
void test_page_rank(G& g, size_t V)
{
  typedef graph_view<G> graph_view_t;
  graph_view_t vgraph(g);

  one_print("Testing PageRank...\t\t\t");

  stapl::page_rank(vgraph, SITER, 0.85);

  bool err = check_result(vgraph);
  one_print(err);
  stapl::rmi_fence();

  one_print("Testing Hierarchical PageRank...\t\t\t");

  auto h = stapl::create_level_machine(vgraph);
  auto h_exec_policy = stapl::sgl::hierarchical_policy<graph_view_t>{h};
  stapl::page_rank(stapl::sgl::execution_policy<graph_view_t>{h_exec_policy},
                   vgraph, SITER, 0.85);

  err = check_result(vgraph);
  one_print(err);
  stapl::rmi_fence();

  one_print("Testing Hierarchical + Hubs PageRank...\t\t\t");

  // Anything with more than 3 edges is considered a hub
  auto hubs = create_level_hubs(vgraph, 3);
  auto hubs_exec_policy =
    stapl::sgl::hierarchical_hubs_policy<graph_view_t>{h, hubs, 3};
  stapl::page_rank(stapl::sgl::execution_policy<graph_view_t>{hubs_exec_policy},
                   vgraph, SITER, 0.85);

  err = check_result(vgraph);
  one_print(err);
  stapl::rmi_fence();
}


template<typename G>
void time_page_rank(G& g, std::string s, size_t V)
{
  stapl::counter<stapl::default_timer> t, t_map;
  double elapsed = 0.0;
  std::vector<double> t_av;

  typedef graph_view<G> graph_view_t;
  graph_view_t vgraph(g);

  one_print("Testing PageRank...\n");

  for (size_t i=0; i<NEXP; ++i) {
    t.reset(); t.start();
      stapl::page_rank(vgraph, SITER, 0.85);
    elapsed = t.stop(); t_av.push_back(elapsed);
  }

  s= "page_rank_"+s;
  double avg_time = compute_stats(s, t_av);
  if (stapl::get_location_id() == 0)
    std::cout << ">> Performance was: "
              << (((g.num_edges()/2)/avg_time)/1000000)
              << " MTEPS.\n" << std::endl;
  /**/
  stapl::rmi_fence();
}


/*
 * This function calls the graph500 benchmark MPI implementation
 * of the input graph generator, to get the same graph as the
 * benchmark reference (the seeds are the same).
 * Each location generates a subset of the (permuted) vertices,
 * therefore, we need the add-edge methods to async to the correct
 * location. Also, the list is directed (to save space), so we need
 * to add both edges (fwd and rev).
 */

template <typename G>
void generate_benchmark_input(G& g, size_t nv, size_t nedges)
{
  // uint64_t seed1 = 2, seed2 = 3;
  // const double initiator[4] = {.57, .19, .19, .05};
  // int64_t* edges;
  size_t  nedges_64 = nedges;

  /* Make the raw graph edges. */
  // make_graph(SCALE, nedges_64, seed1, seed2, initiator,
  //            &nedges_64, &edges);

  nedges_64 = nedges_64/stapl::get_num_locations();
  vector<size_t> edges(2*nedges_64);

  for (size_t i=0; i<nedges_64; ++i) {
    size_t s = rand()%(nv-1);
    size_t t = rand()%(nv-1);
    edges[2*i]   = s;
    edges[2*i+1] = t;
  }

  stapl::counter<stapl::default_timer> add_edges_time;
  {
    add_edge_aggregator<G, false> add_edge_aggr(&g, MAX_EDGE_AGGR_MSG_SZ);
    size_t s,t;

    add_edges_time.start();
    for (size_t i=0; i<edges.size()/2; ++i) {
      s = edges[2*i+0]; t = edges[2*i+1];
      if (s == (uint64_t)(-1) || t == (uint64_t)(-1) || s == t)
        continue;
      else {
        add_edge_aggr.add(typename G::edge_descriptor(s,t));
        add_edge_aggr.add(typename G::edge_descriptor(t,s));
      }
    }
  }
  stapl::rmi_fence();

  // free the edges allocated by make_graph;
  // free(edges);
  stapl::rmi_fence();

  if (stapl::get_location_id()==0) {
    std::cout << "pGraph Construction Time: " << add_edges_time.stop()
              << std::endl;
  }
}

void graph_test_static(size_t nv, size_t ef, size_t bs, bool time=false)
{
  using PGR_static = stapl::multigraph<
    stapl::properties::page_rank_property<float>
  >;

  PGR_static p_static(nv); // for RMAT

  if (time) {
    generate_benchmark_input(p_static, nv, nv*ef);
    if (stapl::get_location_id()==0) {
      cout << "Num vertices: " << p_static.num_vertices() << '\n'
           << "Num edges: " << p_static.num_edges() << std::endl;
    }
    stapl::rmi_fence();
    time_page_rank(p_static, "static", nv);
  } else {
    size_t nx = nv/4;
    size_t ny = 4;
    typedef graph_view<PGR_static> graph_view_t;
    graph_view_t gvw(p_static);
    gvw = generators::make_torus<graph_view_t>(gvw, nx, ny, false);
    rmi_fence();

    test_page_rank(p_static, nv);
  }
}


stapl::exit_code stapl_main(int argc, char* argv[])
{
  size_t bs, ef, nv;
  if (argc > 1) {
    SCALE = atol(argv[1]);
    ef = atol(argv[2]);
  } else {
    cout<<"Usage: exe scale edgefactor.\nusing scale=12 ef=16 by default\n";
    SCALE=12;
    ef=16;
  }

  bool time = false;
  for (int i = 1; i < argc; i++) {
    if (!strcmp("--miniterations", argv[i]))
      NEXP = atoi(argv[i+1]);
    if (!strcmp("--stapliter", argv[i]))
      SITER = atoi(argv[i+1]);
    if (!strcmp("--max_msg_sz", argv[i]))
      MAX_AGGR_MSG_SZ = atoi(argv[i+1]);
    if (!strcmp("--max_edge_msg_sz", argv[i]))
      MAX_EDGE_AGGR_MSG_SZ = atoi(argv[i+1]);
    if (!strcmp("--time", argv[i]))
      time = true;
  }

  nv = pow(2, SCALE);

  bs = nv / stapl::get_num_locations();
  graph_test_static(nv,ef,bs,time);

  return EXIT_SUCCESS;
}

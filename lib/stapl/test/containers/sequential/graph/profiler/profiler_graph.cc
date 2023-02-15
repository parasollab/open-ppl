/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <limits> //currently needed by generate_graph
#include <stapl/containers/sequential/graph/directed_preds_graph.h> //this needs to be before the other includes, for some reason

// Supress deprecated warnings in boost/graph/adjacency_list.hpp (uses auto_ptr)
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif

#include <boost/graph/adjacency_list.hpp>

#ifdef __clang__
#pragma clang diagnostic pop
#endif

#include <stapl/containers/sequential/graph/graph.h>
#include <stapl/containers/sequential/graph/profile/bgl_profilers.h>

using namespace stapl::sequential;

#include <stapl/containers/sequential/graph/profile/stapl_graph_profilers.h>
#include <stapl/containers/sequential/graph/algorithms/graph_generators.h>

using namespace stapl;
using namespace boost;
using stapl::DIRECTED;
using stapl::UNDIRECTED;
using stapl::MULTIEDGES;
using stapl::NONMULTIEDGES;
using std::cout;
using std::endl;

//TODO: Fill out the help text.
void print_help_message() {
  cout<<"The program must be executed with at least 1 integer argument.\n";
  cout<<"./<executable> T [optional args]\n";
  cout<<"T determines the type of test to be performed.\n";

  profile_config::print_help_message();
  return;
}

template<typename BGL_GRAPH>
void bgl_methods_profiler_suite(profile_config& config) {
  BGL_GRAPH* bgptr = new BGL_GRAPH;

  add_vertex_profiler_bgl<BGL_GRAPH> avpbgl("test",bgptr,config);
  avpbgl.collect_profile();
  avpbgl.report();

  delete bgptr;
  bgptr = new BGL_GRAPH;

  delete_vertex_profiler_bgl<BGL_GRAPH> dvpbgl("test",bgptr,config);
  dvpbgl.collect_profile();
  dvpbgl.report();

  delete bgptr;
  bgptr = new BGL_GRAPH;

  add_edge_profiler_bgl<BGL_GRAPH> aepbgl("test",bgptr,config);
  aepbgl.collect_profile();
  aepbgl.report();

  delete bgptr;
  bgptr = new BGL_GRAPH;

  delete_edge_profiler_bgl<BGL_GRAPH> depbgl("test",bgptr,config);
  depbgl.collect_profile();
  depbgl.report();

  delete bgptr;

  return;
}

template<typename STAPL_GRAPH>
void stapl_methods_profiler_suite(profile_config& config) {
  STAPL_GRAPH* sgptr = new STAPL_GRAPH;

  add_vertex_profiler<STAPL_GRAPH> avpstapl("test",sgptr,config);
  avpstapl.collect_profile();
  avpstapl.report();

  delete sgptr;
  sgptr = new STAPL_GRAPH;

  delete_vertex_profiler<STAPL_GRAPH> dvpstapl("test",sgptr,config);
  dvpstapl.collect_profile();
  dvpstapl.report();

  delete sgptr;
  sgptr = new STAPL_GRAPH;

  add_edge_profiler<STAPL_GRAPH> aepstapl("test",sgptr,config);
  aepstapl.collect_profile();
  aepstapl.report();

  delete sgptr;
  sgptr = new STAPL_GRAPH;

  delete_edge_profiler<STAPL_GRAPH> depstapl("test",sgptr,config);
  depstapl.collect_profile();
  depstapl.report();

  delete sgptr;

  return;
}


/**
 * Configure and run experiments comparing the basic graph methods of
 * BGL and SSGL:
 *
 * add_vertex()
 * clear_vertex() + remove_vertex() vs. delete_vertex()
 * add_edge()
 * remove/delete_edge();
 *
 * Note that we pre-generate a graph using SSGL, then store it in memory
 * in a 'collapsed' (vectors of ints) form. We use this collapsed graph or
 * blueprint to ensure that we add/delete the same vertices and edges in
 * both library's set of experiments.
 *
 * We could store this graph blueprint externally as is done in the
 * algorithm experiments, but this doesn't seem necessary since graph
 * sizes don't need to be very large when running these experiments on
 * Hydra, as was originally done.
 *
 * If it becomes necessary to store the pre-generated graphs in external
 * memory instead, the profilers and possibly the utilities will need to
 * be modified in order to do this.
 */
int main(int argc,char** argv) {
  size_t T = 0; //test type; determines what type of graph to use.
  if(argc >= 2) T = atoi(argv[1]);
  else {
    print_help_message();
    return 0;
  }

  profile_config config;
  if(T>0 && T<17) {
    config.configure(argc,argv); //parse the command line
    stapl::sequential::graph<DIRECTED,MULTIEDGES> g;
    build_graph(g,config);

    //===========WARNING==============
    //We store the graph in internal memory since the methods experiments
    //generally don't require large graphs to get good resolution on Hydra.
    //But if the methods experiments were to ever require external storage,
    //just note that many of the profilers will need to be modified to take
    //note of this; they use config.m_verts.size() and config.m_edges.size()
    //when acquiring graph info.
    config.store_graph(g); //stores a collapsed version of g.
    config.build_deletion_lists();
    g.clear();
  }
  else {
    print_help_message();
    return 0;
  }

  cout<<"Benchmark: STAPL Graph vs. BGL"<<endl<<endl;
  //==================================================================
  // Begin experiment cases
  //==================================================================
  if(T==1) {
    typedef adjacency_list<vecS, vecS, directedS> BGL_TYPE;
    typedef stapl::sequential::graph<DIRECTED,MULTIEDGES>     SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\n";
    cout<<"Vertex Storage: Vector\nDirected Edges\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==2) {
    typedef property<vertex_name_t, int>             bgl_vp_t;
    typedef property<edge_weight_t, int>             bgl_ep_t;
    typedef adjacency_list<vecS, vecS, directedS,
                           bgl_vp_t, bgl_ep_t>       BGL_TYPE;

    typedef stapl::sequential::graph<DIRECTED, MULTIEDGES, int> SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"Vector\nDirected Edges\nInteger properties on vertices and edges.\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==3) {
    typedef adjacency_list<vecS, vecS, directedS> BGL_TYPE;
    typedef stapl::sequential::graph<DIRECTED,NONMULTIEDGES>  SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"Vector\nDirected Edges\nSSGL Nonmultiedges\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==4) {
    typedef property<vertex_name_t, int>             bgl_vp_t;
    typedef property<edge_weight_t, int>             bgl_ep_t;
    typedef adjacency_list<vecS, vecS, directedS,
                           bgl_vp_t, bgl_ep_t>       BGL_TYPE;

    typedef stapl::sequential::graph<DIRECTED, NONMULTIEDGES, int> SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"Vector\nDirected Edges\nInteger properties on vertices and ";
    cout<<"edges.\nSSGL Nonmultiedges\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==5) {
    typedef adjacency_list<vecS, listS, directedS> BGL_TYPE;

    typedef stapl::properties::no_property NP;
    typedef stapl::sequential::adj_graph_traits_list_storage<DIRECTED, MULTIEDGES,
                                                             NP, NP>  list_traits;

    typedef stapl::sequential::graph<DIRECTED, MULTIEDGES, NP, NP, list_traits> SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"List/Smart List\nDirected Edges\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==6) {
    typedef property<vertex_name_t, int>             bgl_vp_t;
    typedef property<edge_weight_t, int>             bgl_ep_t;
    typedef adjacency_list<vecS, listS, directedS,
                           bgl_vp_t, bgl_ep_t>       BGL_TYPE;

    typedef stapl::sequential::adj_graph_traits_list_storage<DIRECTED, MULTIEDGES,
                                                             int, int>  list_traits;
    typedef stapl::sequential::graph<DIRECTED, MULTIEDGES, int, int, list_traits> SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"List/Smart List\nDirected Edges\nInteger properties on vertices ";
    cout<<"and edges.\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==7) {
    typedef adjacency_list<vecS, listS, directedS> BGL_TYPE;

    typedef stapl::properties::no_property NP;
    typedef stapl::sequential::adj_graph_traits_list_storage<DIRECTED, NONMULTIEDGES,
                                                             NP, NP>  list_traits;
    typedef stapl::sequential::graph<DIRECTED, NONMULTIEDGES,
                         NP, NP, list_traits>             SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"List/Smart List\nDirected Edges\nNo properties\nSSGL Nonmultiedges\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==8) {
    typedef property<vertex_name_t, int>             bgl_vp_t;
    typedef property<edge_weight_t, int>             bgl_ep_t;
    typedef adjacency_list<vecS, listS, directedS,
                           bgl_vp_t, bgl_ep_t>       BGL_TYPE;

    typedef stapl::sequential::adj_graph_traits_list_storage<DIRECTED, NONMULTIEDGES,
                                                             int, int>  list_traits;
    typedef stapl::sequential::graph<DIRECTED, NONMULTIEDGES,
                         int, int, list_traits>             SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"List/Smart List\nDirected Edges\nInteger properties on vertices";
    cout<<" and edges\nSSGL Nonmultiedges\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==9) {
    typedef adjacency_list<vecS, vecS, undirectedS> BGL_TYPE;
    typedef stapl::sequential::graph<UNDIRECTED,MULTIEDGES>     SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\n";
    cout<<"Vertex Storage: Vector\nUndirected Edges\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==10) {
    typedef property<vertex_name_t, int>             bgl_vp_t;
    typedef property<edge_weight_t, int>             bgl_ep_t;
    typedef adjacency_list<vecS, vecS, undirectedS,
                           bgl_vp_t, bgl_ep_t>       BGL_TYPE;

    typedef stapl::sequential::graph<UNDIRECTED, MULTIEDGES, int> SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"Vector\nUndirected Edges\nInteger properties on vertices and edges.\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==11) {
    typedef adjacency_list<vecS, vecS, undirectedS> BGL_TYPE;
    typedef stapl::sequential::graph<UNDIRECTED,NONMULTIEDGES>  SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"Vector\nUndirected Edges\nSSGL Nonmultiedges\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==12) {
    typedef property<vertex_name_t, int>             bgl_vp_t;
    typedef property<edge_weight_t, int>             bgl_ep_t;
    typedef adjacency_list<vecS, vecS, undirectedS,
                           bgl_vp_t, bgl_ep_t>       BGL_TYPE;

    typedef stapl::sequential::graph<UNDIRECTED, NONMULTIEDGES, int> SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"Vector\nUndirected Edges\nInteger properties on vertices and ";
    cout<<"edges.\nSSGL Nonmultiedges\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==13) {
    typedef adjacency_list<vecS, listS, undirectedS> BGL_TYPE;

    typedef stapl::properties::no_property NP;
    typedef stapl::sequential::adj_graph_traits_list_storage<UNDIRECTED, MULTIEDGES,
                                                             NP, NP>  list_traits;

    typedef stapl::sequential::graph<UNDIRECTED, MULTIEDGES, NP, NP, list_traits> SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"List/Smart List\nUndirected Edges\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==14) {
    typedef property<vertex_name_t, int>             bgl_vp_t;
    typedef property<edge_weight_t, int>             bgl_ep_t;
    typedef adjacency_list<vecS, listS, undirectedS,
                           bgl_vp_t, bgl_ep_t>       BGL_TYPE;

    typedef stapl::sequential::adj_graph_traits_list_storage<UNDIRECTED, MULTIEDGES,
                                                             int, int>  list_traits;
    typedef stapl::sequential::graph<UNDIRECTED, MULTIEDGES, int, int, list_traits> SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"List/Smart List\nUndirected Edges\nInteger properties on vertices ";
    cout<<"and edges.\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==15) {
    typedef adjacency_list<vecS, listS, undirectedS> BGL_TYPE;

    typedef stapl::properties::no_property NP;
    typedef stapl::sequential::adj_graph_traits_list_storage<UNDIRECTED, NONMULTIEDGES,
                                                             NP, NP>  list_traits;
    typedef stapl::sequential::graph<UNDIRECTED, NONMULTIEDGES,
                         NP, NP, list_traits>             SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"List/Smart List\nUndirected Edges\nNo properties\nSSGL Nonmultiedges\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  if(T==16) {
    typedef property<vertex_name_t, int>             bgl_vp_t;
    typedef property<edge_weight_t, int>             bgl_ep_t;
    typedef adjacency_list<vecS, listS, undirectedS,
                           bgl_vp_t, bgl_ep_t>       BGL_TYPE;

    typedef stapl::sequential::adj_graph_traits_list_storage<UNDIRECTED, NONMULTIEDGES,
                                                             int, int>  list_traits;
    typedef stapl::sequential::graph<UNDIRECTED, NONMULTIEDGES,
                         int, int, list_traits>             SSGL_TYPE;

    cout<<"Profiler Suite: Methods\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"List/Smart List\nUndirected Edges\nInteger properties on vertices";
    cout<<" and edges\nSSGL Nonmultiedges\n\n";
    bgl_methods_profiler_suite<BGL_TYPE>(config);
    stapl_methods_profiler_suite<SSGL_TYPE>(config);
  }

  cout<<"That's all folks!"<<endl;
  return 0;
}


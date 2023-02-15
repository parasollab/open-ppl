/*
// Copyright (c) 2000-2009, Texas Engineering Experiment Station (TEES), a
// component of the Texas A&M University System.

// All rights reserved.

// The information and source code contained herein is the exclusive
// property of TEES and may not be disclosed, examined or reproduced
// in whole or in part without explicit written authorization from TEES.
*/

#include <limits> //currently needed by generate_graph
#include <stapl/containers/sequential/graph/graph.h>
#include <stapl/containers/sequential/graph/profile/bgl_profilers.h>
#include <stapl/containers/sequential/graph/profile/profile_utils.h>

using namespace stapl::sequential;

#include <stapl/containers/sequential/graph/algorithms/dijkstra.h>
#include <stapl/containers/sequential/graph/profile/stapl_graph_profilers.h>
#include <boost/graph/adjacency_list.hpp>
#include <stapl/containers/sequential/graph/algorithms/graph_generators.h>

using namespace boost;
using namespace stapl;
using stapl::UNDIRECTED;
using stapl::DIRECTED;
using stapl::NONMULTIEDGES;
using stapl::MULTIEDGES;
using std::cout;
using std::cerr;
using std::endl;

void print_help_message() {
  cout<<"The program must be executed with at least 1 integer argument.\n";
  cout<<"./<executable> T [optional args]\n";
  cout<<"T determines the algorithms tested, graph directedness, and ";
  cout<<"algorithm data location.\n";
  cout<<"Note that, in general, you want to use the optional args.\n";

  cout<<"Test types:\nTests 1-4 are for Directed graphs.\n";
  cout<<"1 --- BFS/DFS with data stored internally.\n";
  cout<<"2 --- BFS/DFS with data stored externally.\n";
  cout<<"3 --- Dijkstra with data stored internally.\n";
  cout<<"4 --- Dijkstra with data stored externally.\n\n";
  cout<<"Tests 5-8 are for Undirected graphs.\n";
  cout<<"5 --- BFS/DFS with data stored internally.\n";
  cout<<"6 --- BFS/DFS with data stored externally.\n";
  cout<<"7 --- Dijkstra with data stored internally.\n";
  cout<<"8 --- Dijkstra with data stored externally.\n";

  profile_config::print_help_message();
  return;
}

//the propertize() methods assume the graphs have size_t properties.
template<typename BGL_GRAPH, typename STAPL_GRAPH>
void dijkstra_profiler_suite(std::string filename, int argc, char** argv) {
  typedef typename BGL_GRAPH::vertex_descriptor   bgl_vd_t;
  typedef typename STAPL_GRAPH::vertex_descriptor stapl_vd_t;
  typedef typename std::vector<bgl_vd_t>          bgl_verts_t;
  typedef typename std::vector<stapl_vd_t>        stapl_verts_t;

  size_t seed = time(NULL);
  cout<<"Propertizing the graphs with seed: "<<seed<<endl;

  BGL_GRAPH* bgptr = new BGL_GRAPH;
  bgl_verts_t bgl_verts = read_from_file_bgl(*bgptr, filename);
  propertize_bgl(*bgptr,seed);

  dijkstra_profiler_bgl<BGL_GRAPH> dbgl("test",bgptr,argc,argv);
  dbgl.collect_profile();
  dbgl.report();

  bgptr->clear();
  delete bgptr;

  STAPL_GRAPH* sgptr = new STAPL_GRAPH;
  stapl_verts_t stapl_verts = read_from_file(*sgptr, filename);
  propertize(*sgptr,seed);

  dijkstra_profiler<STAPL_GRAPH> dstapl("test",sgptr,argc,argv);
  dstapl.collect_profile();
  dstapl.report();

  sgptr->clear();
  delete sgptr;
}

template<typename BGL_GRAPH, typename STAPL_GRAPH>
void dsssp_internal_profiler_suite(std::string filename, int argc, char** argv) {
  typedef typename BGL_GRAPH::vertex_descriptor     bgl_vd_t;
  typedef typename STAPL_GRAPH::vertex_descriptor   stapl_vd_t;
  typedef typename std::vector<bgl_vd_t>        bgl_verts_t;
  typedef typename std::vector<stapl_vd_t>      stapl_verts_t;

  size_t seed = time(NULL);
  cout<<"Propertizing the graphs with seed: "<<seed<<endl;

  BGL_GRAPH* bgptr = new BGL_GRAPH;
  read_from_file_bgl(*bgptr, filename);
  propertize_bgl(*bgptr,seed);

  dsssp_in_profiler_bgl<BGL_GRAPH> dbgl("test",bgptr,argc,argv);
  dbgl.collect_profile();
  dbgl.report();

  bgptr->clear();
  delete bgptr;

  STAPL_GRAPH* sgptr = new STAPL_GRAPH;
  read_from_file(*sgptr, filename);
  propertize(*sgptr,seed);

  dsssp_in_profiler<STAPL_GRAPH> dstapl("test",sgptr,argc,argv);
  dstapl.collect_profile();
  dstapl.report();

  sgptr->clear();
  delete sgptr;
}

template<typename BGL_GRAPH, typename STAPL_GRAPH>
void algo_profiler_suite(std::string filename, int argc, char** argv) {
  BGL_GRAPH* bgptr = new BGL_GRAPH;
  read_from_file_bgl(*bgptr, filename);

  bfs_profiler_bgl<BGL_GRAPH> bfsbgl("test",bgptr,argc,argv);
  bfsbgl.collect_profile();
  bfsbgl.report();

  dfs_profiler_bgl<BGL_GRAPH> dfsbgl("test",bgptr,argc,argv);
  dfsbgl.collect_profile();
  dfsbgl.report();

  bgptr->clear();
  delete bgptr;

  STAPL_GRAPH* sgptr = new STAPL_GRAPH;
  read_from_file(*sgptr, filename);

  bfs_profiler<STAPL_GRAPH> bfsstapl("test",sgptr,argc,argv);
  bfsstapl.collect_profile();
  bfsstapl.report();

  dfs_profiler<STAPL_GRAPH> dfsstapl("test",sgptr,argc,argv);
  dfsstapl.collect_profile();
  dfsstapl.report();

  sgptr->clear();
  delete sgptr;
}

template<typename BGL_GRAPH, typename STAPL_GRAPH>
void algo_internal_profiler_suite(std::string filename, int argc, char** argv) {
  BGL_GRAPH* bgptr = new BGL_GRAPH;
  read_from_file_bgl(*bgptr, filename);

  bfs_internal_profiler_bgl<BGL_GRAPH> bfsbgl("test",bgptr,argc,argv);
  bfsbgl.collect_profile();
  bfsbgl.report();

  dfs_internal_profiler_bgl<BGL_GRAPH> dfsbgl("test",bgptr,argc,argv);
  dfsbgl.collect_profile();
  dfsbgl.report();

  bgptr->clear();
  delete bgptr;

  STAPL_GRAPH* sgptr = new STAPL_GRAPH;
  read_from_file(*sgptr, filename);

  bfs_internal_profiler<STAPL_GRAPH> bfsstapl("test",sgptr,argc,argv);
  bfsstapl.collect_profile();
  bfsstapl.report();

  dfs_internal_profiler<STAPL_GRAPH> dfsstapl("test",sgptr,argc,argv);
  dfsstapl.collect_profile();
  dfsstapl.report();

  sgptr->clear();
  delete sgptr;
}

template<typename BGL_GRAPH, typename STAPL_GRAPH>
void traversal_profiler_suite(std::string filename, int argc, char** argv) {
  BGL_GRAPH* bgptr = new BGL_GRAPH;
  read_from_file_bgl(*bgptr, filename);

  traversal_profiler_bgl<BGL_GRAPH> tpbgl("test",bgptr,argc,argv);
  tpbgl.collect_profile();
  tpbgl.report();

  bgptr->clear();
  delete bgptr;

  STAPL_GRAPH* sgptr = new STAPL_GRAPH;
  read_from_file(*sgptr, filename);

  traversal_profiler<STAPL_GRAPH> tpssgl("test",sgptr,argc,argv);
  tpssgl.collect_profile();
  tpssgl.report();

  sgptr->clear();
  delete sgptr;
}


/**
 * Configure and run experiments on BGL's and SSGL's implementations of
 * BFS, DFS, and Dijkstra's algorithm.
 *
 * A graph is pre-generated and then stored in external memory. This is
 * done so that we can guarantee that the graph build for both BGL and
 * SSGL are reading from the same input, which helps us ensure that the
 * algorithm profilers are always operating on the same input graph.
 *
 * For information on each test type (case), just read the cases
 * indvidually or execute the binary without any arguments.
 *
 * Furthermore, the graphs tend to become very large, so storing a
 * collapsed/blueprint graph in internal memory would be prohibitive.
 * However, if this ever becomes necessary, all that needs to be done
 * is to have the config object store the graph instead of using
 * write_to_file and the profiler suite methods defined above need to
 * take the config object (by reference!) and use it to rebuild the
 * graph instead of reading from external memory.
 */
int main(int argc,char** argv) {
  size_t T=0;     //determines graph attributes and algo. suite
  if(argc >= 2) T=atoi(argv[1]);
  else {
    print_help_message();
    return 0;
  }

  //Build the graph
  profile_config config;
  std::string filename = "graph.txt";
  if((T>0 && T<11) || T==1337) {
    config.configure(argc,argv);
    stapl::sequential::graph<DIRECTED,MULTIEDGES> g;
    build_graph(g,config);
    write_to_file(g,filename);
  }
  else {
    print_help_message();
    return 0;
  }

  cout<<"Benchmark: STAPL Graph vs. BGL"<<endl<<endl;
  //==================================================================
  // Begin test cases
  //==================================================================
  if(T==1) {
    typedef boost::property<boost::vertex_color_t,
                            boost::default_color_type>  bgl_vp_t;
    typedef adjacency_list<vecS, vecS, directedS,
                           bgl_vp_t>                    BGL_TYPE;
    typedef stapl::sequential::graph<DIRECTED,MULTIEDGES,int>       STAPL_TYPE;

    cout<<"Algo. Profiler Suite:\nVertex Storage: Vector\n";
    cout<<"Edge Storage: Vector\nDirected Edges\nProperties stored internally.\n\n";
    algo_internal_profiler_suite<BGL_TYPE,STAPL_TYPE>(filename,argc,argv);
  }

  if(T==2) {
    typedef adjacency_list<vecS, vecS, directedS> BGL_TYPE;
    typedef stapl::sequential::graph<DIRECTED,MULTIEDGES>     STAPL_TYPE;

    cout<<"Algo. Profiler Suite:\nVertex Storage: Vector\n";
    cout<<"Edge Storage: Vector\nDirected Edges\nProperties stored externally.\n\n";
    algo_profiler_suite<BGL_TYPE,STAPL_TYPE>(filename,argc,argv);
  }

  if(T==3) {
    //The vertex_name_t property is to store the vertex's parent given by
    //Dijkstra's algo. It has nothing to do with any kind of naming.
    typedef property<vertex_color_t, default_color_type>    bgl_vp_t3;
    typedef property<vertex_name_t, size_t, bgl_vp_t3>      bgl_vp_t2;
    typedef property<vertex_distance_t, size_t, bgl_vp_t2>  bgl_vp_t;
    typedef property<edge_weight_t, size_t>                 bgl_ep_t;
    typedef adjacency_list<vecS, vecS, directedS,
                           bgl_vp_t, bgl_ep_t>              BGL_TYPE;

    typedef size_t                                          stapl_ep_t;
    typedef stapl::dijkstra_property<size_t,stapl_ep_t,int> stapl_vp_t;
    typedef stapl::sequential::graph<DIRECTED,MULTIEDGES,
                                     stapl_vp_t, stapl_ep_t>            STAPL_TYPE;

    cout<<"Dijkstra Profiler Suite:\nVertex Storage: Vector\n";
    cout<<"Edge Storage: Vector\nDirected Edges\nProperties stored internally.\n\n";
    dsssp_internal_profiler_suite<BGL_TYPE, STAPL_TYPE>(filename,argc,argv);
  }

  if(T==4) {
    typedef adjacency_list<vecS, vecS, directedS, boost::no_property,
                           property<edge_weight_t, size_t> > BGL_TYPE;
    typedef stapl::sequential::graph<DIRECTED,MULTIEDGES,
                                     stapl::properties::no_property,
                                     size_t>  STAPL_TYPE;

    cout<<"Dijkstra Profiler Suite:\nVertex Storage: Vector\n";
    cout<<"Edge Storage: Vector\nDirected Edges\nProperties stored externally.\n\n";
    dijkstra_profiler_suite<BGL_TYPE, STAPL_TYPE>(filename,argc,argv);
  }

  if(T==5) {
    typedef boost::property<boost::vertex_color_t,
                            boost::default_color_type>  bgl_vp_t;
    typedef adjacency_list<vecS, vecS, undirectedS,
                           bgl_vp_t>                    BGL_TYPE;
    typedef stapl::sequential::graph<UNDIRECTED,MULTIEDGES,int>     STAPL_TYPE;

    cout<<"Algo. Profiler Suite:\nVertex Storage: Vector\n";
    cout<<"Edge Storage: Vector\nUndirected Edges\nProperties stored internally.\n\n";
    algo_internal_profiler_suite<BGL_TYPE,STAPL_TYPE>(filename,argc,argv);
  }

  if(T==6) {
    typedef adjacency_list<vecS, vecS, undirectedS> BGL_TYPE;
    typedef stapl::sequential::graph<UNDIRECTED,MULTIEDGES>     STAPL_TYPE;

    cout<<"Algo. Profiler Suite:\nVertex Storage: Vector\n";
    cout<<"Edge Storage: Vector\nUndirected Edges\nProperties stored externally.\n\n";
    algo_profiler_suite<BGL_TYPE,STAPL_TYPE>(filename,argc,argv);
  }

  if(T==7) {
    //The vertex_name_t property is to store the vertex's parent given by
    //Dijkstra's algo. It has nothing to do with any kind of naming.
    typedef property<vertex_color_t, default_color_type>    bgl_vp_t3;
    typedef property<vertex_name_t, size_t, bgl_vp_t3>      bgl_vp_t2;
    typedef property<vertex_distance_t, size_t, bgl_vp_t2>  bgl_vp_t;
    typedef property<edge_weight_t, size_t>                 bgl_ep_t;
    typedef adjacency_list<vecS, vecS, undirectedS,
                           bgl_vp_t, bgl_ep_t>              BGL_TYPE;

    typedef size_t                                          stapl_ep_t;
    typedef stapl::dijkstra_property<size_t,stapl_ep_t,int> stapl_vp_t;
    typedef stapl::sequential::graph<UNDIRECTED,MULTIEDGES,
                                     stapl_vp_t, stapl_ep_t>            STAPL_TYPE;

    cout<<"Dijkstra Profiler Suite:\nVertex Storage: Vector\n";
    cout<<"Edge Storage: Vector\nUndirected Edges\nProperties stored internally.\n\n";
    dsssp_internal_profiler_suite<BGL_TYPE, STAPL_TYPE>(filename,argc,argv);
  }

  if(T==8) {
    typedef adjacency_list<vecS, vecS, undirectedS, boost::no_property,
                           property<edge_weight_t, size_t> > BGL_TYPE;
    typedef stapl::sequential::graph<UNDIRECTED,MULTIEDGES,
                                     stapl::properties::no_property,
                                     size_t>  STAPL_TYPE;

    cout<<"Dijkstra Profiler Suite:\nVertex Storage: Vector\n";
    cout<<"Edge Storage: Vector\nUndirected Edges\nProperties stored externally.\n\n";
    dijkstra_profiler_suite<BGL_TYPE, STAPL_TYPE>(filename,argc,argv);
  }

  if(T==9) {
    typedef adjacency_list<vecS, vecS, directedS> BGL_TYPE;
    typedef stapl::sequential::graph<DIRECTED,MULTIEDGES>     STAPL_TYPE;

    cout<<"Traversal Profiler Suite:\nVertex Storage: Vector\n";
    cout<<"Edge Storage: Vector\nDirected Edges\nNo properties.\n\n";
    traversal_profiler_suite<BGL_TYPE, STAPL_TYPE>(filename,argc,argv);
  }

  if(T==10) {
    typedef property<vertex_name_t, int>          vp_t;
    typedef property<edge_weight_t, int>          ep_t;
    typedef adjacency_list<vecS, vecS, directedS, vp_t, ep_t> BGL_TYPE;
    typedef stapl::sequential::graph<DIRECTED,MULTIEDGES,int,int> STAPL_TYPE;

    cout<<"Traversal Profiler Suite:\nVertex Storage: Vector\n";
    cout<<"Edge Storage: Vector\nDirected Edges\nInteger properties.\n\n";
    traversal_profiler_suite<BGL_TYPE, STAPL_TYPE>(filename,argc,argv);
  }
  //==================================================================
  // Smart vector storage in SSGL
  //==================================================================
  if(T==42) {
    typedef boost::property<boost::vertex_color_t,
                            boost::default_color_type>  bgl_vp_t;
    typedef adjacency_list<vecS, vecS, directedS,
                           bgl_vp_t>                    BGL_TYPE;

    typedef stapl::sequential::adj_graph_traits_vector_storage<
      DIRECTED, MULTIEDGES, int, stapl::properties::no_property> vector_trait;
    typedef stapl::sequential::graph<DIRECTED,
                                     MULTIEDGES,
                                     int,
                                     stapl::properties::no_property,
                                     vector_trait> STAPL_TYPE;
    typedef stapl::sequential::graph<DIRECTED,MULTIEDGES> BUILDER_T;

    cout<<"Profiler Suite:\nEdge Storage: Vector\nVertex Storage: ";
    cout<<"Vector/Smart Vector\nDirected Edges\n\n";
    algo_internal_profiler_suite<BGL_TYPE,STAPL_TYPE>(filename,argc,argv);
  }

  cout<<"That's all folks!"<<endl;

  return 0;
}


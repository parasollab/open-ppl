// cont/dygraf.cc

#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/containers/graph/dynamic_graph.hpp>
#include <stapl/containers/graph/views/graph_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

// CODE - finish

int prime[] = { 2,  3,  5,  7, 11, 13, 17, 19, 23, 29,
               31, 37, 41, 43, 47, 53, 59, 61, 67, 71 };

int fibo[] = { 1, 2, 3, 4, 8, 13, 21, 34, 55, 89,
             144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946 };

typedef int val_tp;
typedef stapl::dynamic_graph<stapl::UNDIRECTED, stapl::NONMULTIEDGES,
                             val_tp, val_tp> dygraf_int_tp;
typedef stapl::graph_view<dygraf_int_tp> dygraf_int_vw_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                             val_tp, val_tp> stgraf_int_tp;
typedef stapl::graph_view<stgraf_int_tp> stgraf_int_vw_tp;

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

struct init_prop_wf {
  typedef void result_type;
  template <typename Vertex>
  result_type operator()(Vertex vtx) {
    int i = 0;
    for ( auto edge_it = vtx.begin(); edge_it != vtx.end(); ++edge_it ) {
      (*edge_it).property() = prime[i++];
    }
  }
};
 
struct add_vert_wf {
  typedef void result_type;
  template <typename Property, typename Graph>
  result_type operator()(Property prop, Graph graf) {
    graf.add_vertex(prop);
  }
};
 
struct add_edge_wf {
  typedef void result_type;
  template <typename Vertex, typename Count, typename Graph>
  result_type operator()(Vertex vtx, Count cnt, Graph graf) {
    size_t i = 0;
    size_t src = 0;
    for ( auto vtx_it = vtx.begin(); vtx_it != vtx.end(); ++vtx_it ) {
      for ( size_t j = 0; j < cnt/2; j++ ) {
        size_t dest = 0;
        do {
          dest = rand() % cnt;
        } while ( src != dest );
        auto ed = graf.add_edge( src, dest, prime[i%20]/2 );
        if (numeric_limits<size_t>::max() == ed.id() ) { // FAILED?
        }
        i++;
      }
      src++;
    }
  }
};

struct update_prop_wf {
  typedef void result_type;
  template <typename Vertex, typename Property>
  result_type operator()(Vertex vtx, Property prop) {
    for ( auto edge_it = vtx.begin(); edge_it != vtx.end(); ++edge_it ) {
      (*edge_it).property() = prop * 100;
    }
  }
};


stapl::exit_code stapl_main(int argc, char **argv) {

  // construct graph
  dygraf_int_tp dg_ct(20);

  // construct view over graph
  dygraf_int_vw_tp dg_vw(dg_ct);

  // initialize vertex properties in parallel
  stapl::map_func( init_prop_wf(),
                   dg_vw );
 
  size_t count = dg_vw.num_vertices();

  // add graph vertices to dynamic graph in parallel
  vec_int_tp prop_vec(10);
  vec_int_vw_tp prop_vec_vw(prop_vec);
  stapl::iota(prop_vec_vw, 100 );
  stapl::map_func( add_vert_wf(),
                   prop_vec_vw,
                   stapl::make_repeat_view<dygraf_int_vw_tp>(dg_vw) );
 
  // add edges in parallel
  stapl::map_func( add_edge_wf(),
                   dg_vw, stapl::make_repeat_view(count),
                   stapl::make_repeat_view<dygraf_int_vw_tp>(dg_vw) );

  // process graph elements in parallel
  stapl::map_func( update_prop_wf(),
                   dg_vw, stapl::counting_view<int>(count) );

  // print vertex and edge count
  stapl::stream<ofstream> zout;
  zout.open("refman_grafvw.txt");
  stapl::do_once( msg_val<int>( zout, "Vertices ", dg_vw.num_vertices() ) );
  stapl::do_once( msg_val<int>( zout, "Edges ", dg_vw.num_edges() ) );

  return EXIT_SUCCESS;
}

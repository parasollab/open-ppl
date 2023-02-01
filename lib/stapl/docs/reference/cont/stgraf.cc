// cont/dygraf.cc
 
#include <cstdlib>
#include <stapl/containers/graph/graph.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
 
#include "conthelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                               val_tp, val_tp> 
               stgraf_int_tp;
typedef stapl::graph<stapl::DIRECTED, stapl::NONMULTIEDGES,
                               val_tp, val_tp, 
                     stapl::view_based_partition<distrib_spec>,
                     stapl::view_based_mapper<distrib_spec> >
               stgraf_int_dist_tp;

int prime[20] = { 2,  3,  5,  7, 11, 13, 17, 19, 23, 29,
                 31, 37, 41, 43, 47, 53, 59, 61, 67, 71 };

int fibo[20] = { 1, 2, 3, 4, 8, 13, 21, 34, 55, 89,
               144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946 };


stapl::exit_code stapl_main(int argc, char **argv) {

  size_t size = 20;
  srand(16807);
  // construct container
  stgraf_int_tp g_ct(size);

  // construct container with non-default distribution
  distrib_spec old_dist = stapl::block(size,10);
  stgraf_int_dist_tp h_ct(old_dist);

  // add edges to vertices with sequential loop
  size_t vert_cnt = g_ct.num_vertices();
  size_t k = 0;
  size_t src = 0;
  for ( auto vtx_it = g_ct.begin(); vtx_it != g_ct.end(); ++vtx_it ) {
    for ( size_t j = 0; j < vert_cnt/2; j++ ) {
      size_t dest = 0;
      do {
        dest = rand() % vert_cnt;
      } while ( src != dest );
      auto ed = g_ct.add_edge( src, dest, prime[k%size]/2 );
      if (numeric_limits<size_t>::max() == ed.id() ) { // FAILED
      }
      k++;
    }
    src++;
  }
  stapl::rmi_fence();

  // initialize vertex properties with sequential loop
  size_t i=0;
  for ( auto vtx_it = g_ct.begin(); vtx_it != g_ct.end(); ++vtx_it ) {
    auto vtx = *vtx_it;
    for ( auto edge_it = vtx.begin(); edge_it != vtx.end(); ++edge_it ) {
      (*edge_it).property() = prime[i++];
    }
  }
  size_t j=0;
  for ( auto vtx_it = h_ct.begin(); vtx_it != h_ct.end(); ++vtx_it ) {
    auto vtx = *vtx_it;
    for ( auto edge_it = vtx.begin(); edge_it != vtx.end(); ++edge_it ) {
      (*edge_it).property() = prime[j++]/2;
    }
  }
  
  // redistribute container
  distrib_spec new_dist = stapl::cyclic(size,2);
  h_ct.redistribute(new_dist);

  // print edge and vertex count
  stapl::stream<ofstream> zout;
  zout.open("refman_stgraf.txt");
  stapl::do_once( msg_val<int>( zout, "Vertices ", g_ct.num_vertices() ) );
  stapl::do_once( msg_val<int>( zout, "Edges ", g_ct.num_edges() ) );

  return EXIT_SUCCESS;
}


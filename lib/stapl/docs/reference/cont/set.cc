// cont/set.cc
 
#include <stapl/containers/set/set.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
 
#include "conthelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::set<val_tp> set_int_tp;
typedef stapl::set<val_tp, stapl::less<val_tp>,
        stapl::view_based_partition<distrib_spec>,
        stapl::view_based_mapper<distrib_spec> > set_int_dist_tp;

int prime[20]= { 2,  3,  5,  7, 11, 13, 17, 19, 23, 29,
                31, 37, 41, 43, 47, 53, 59, 61, 67, 71 };

int fibo[20] = { 1, 2, 3, 4, 8, 13, 21, 34, 55, 89,
               144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946 };


stapl::exit_code stapl_main(int argc, char **argv) {

  size_t size = 20;
  // construct container
  set_int_tp s_ct;

  // initialize container using sequential loop
  for( int i=0; i<size; i++ ) {
    s_ct.insert(fibo[i]);
  }
  for( int i=0; i<size; i++ ) {
    s_ct.insert(prime[i]);
  }
  stapl::rmi_fence();

  // find an element
  auto it = s_ct.find(144);
  if ( it != s_ct.end() ) {
    // found
  }

  // print size
  stapl::stream<ofstream> zout;
  zout.open("refman_set.txt");
  stapl::do_once( msg_val<int>( zout, "Size ", s_ct.size() ) );
 
  return EXIT_SUCCESS;
}

// cont/vector.cc
 
#include <stapl/containers/vector/vector.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
 
#include "conthelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector<val_tp,
        stapl::view_based_partition<distrib_spec>,
        stapl::view_based_mapper<distrib_spec> > vec_int_dist_tp;

int prime[20] = { 2,  3,  5,  7, 11, 13, 17, 19, 23, 29,
                 31, 37, 41, 43, 47, 53, 59, 61, 67, 71 };

int fibo[20] = { 1, 2, 3, 4, 8, 13, 21, 34, 55, 89,
               144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946 };


stapl::exit_code stapl_main(int argc, char **argv) {

  size_t size = 20;
  // construct container
  vec_int_tp v_ct(size);

  // construct container with non-default initializer
  vec_int_tp u_ct(size, 314159);

  // construct container with non-default distribution
  distrib_spec old_dist = stapl::block(size,10);
  vec_int_dist_tp w_ct(old_dist);

  // initialize container with sequential loop
  auto v_it= v_ct.begin(); 
  auto w_it= w_ct.begin(); 
  for( size_t i= 0; i<size; i++ ) {
    *v_it++ = prime[i];
    *w_it++ = fibo[i];
  }

  // add elements
  v_ct.push_back(1001);
  v_ct.push_back(2002);
  v_ct.push_back(3003);
  v_ct.flush();

  // redistribute container
  distrib_spec new_dist = stapl::block_cyclic(size,5);
  w_ct.redistribute(new_dist);

  // print container size
  stapl::stream<ofstream> zout;
  zout.open("refman_vector.txt");
  stapl::do_once( msg_val<int>( zout, "Size ", v_ct.size() ) );

  return EXIT_SUCCESS;
}

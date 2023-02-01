// cont/array.cc

#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
 
#include "conthelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::array<val_tp> ary_int_tp;

typedef stapl::array<val_tp,
        stapl::view_based_partition<distrib_spec>,
        stapl::view_based_mapper<distrib_spec> > ary_int_dist_tp;

int prime[20] = { 2,  3,  5,  7, 11, 13, 17, 19, 23, 29,
                 31, 37, 41, 43, 47, 53, 59, 61, 67, 71 };

int fibo[20] = { 1, 2, 3, 4, 8, 13, 21, 34, 55, 89,
               144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946 };


stapl::exit_code stapl_main(int argc, char **argv) {

  size_t size = 20;
  // construct container
  ary_int_tp a_ct(size);

  // construct container with non-default initializer
  ary_int_tp c_ct(size, 314159);

  // construct container with non-default distribution
  distrib_spec old_dist = stapl::block(size,10);
  ary_int_dist_tp b_ct(old_dist);

  // initialize container in sequential loop
  auto a_it= a_ct.begin();
  auto b_it= b_ct.begin(); 
  for( size_t i=0; i<size; i++ ) {
    *a_it++ = prime[i];
    *b_it++ = fibo[i];
  }
  
  // redistribute container
  distrib_spec new_dist = stapl::block_cyclic(size,5);
  b_ct.redistribute(new_dist);

  // print container size
  stapl::stream<ofstream> zout;
  zout.open("refman_array.txt");
  stapl::do_once( msg_val<int>( zout, "Size ", a_ct.size() ) );

  return EXIT_SUCCESS;
}

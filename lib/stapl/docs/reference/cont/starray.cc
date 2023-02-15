// cont/starray.cc
 
#include <stapl/containers/array/static_array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
 
#include "conthelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::static_array<val_tp> ary_int_tp;

int prime[20] = { 2,  3,  5,  7, 11, 13, 17, 19, 23, 29,
                 31, 37, 41, 43, 47, 53, 59, 61, 67, 71 };

int fibo[20] = { 1, 2, 3, 4, 8, 13, 21, 34, 55, 89,
               144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946 };


stapl::exit_code stapl_main(int argc, char **argv) {

  // construct array and view over it
  ary_int_tp a_ct(20);

  // construct container with non-default initializer
  ary_int_tp b_ct(20, 314159);

  // initialize container in sequential loop
  int i=0;
  for( auto it= a_ct.begin(); it != a_ct.end(); it++ ) {
    *it++ = prime[i++];
  }

  // modify element
  a_ct[10] = 0;
  stapl::rmi_fence();

  // print container size
  stapl::stream<ofstream> zout;
  zout.open("refman_array.txt");
  stapl::do_once( msg_val<int>( zout, "Size ", a_ct.size() ) );

  return EXIT_SUCCESS;
}

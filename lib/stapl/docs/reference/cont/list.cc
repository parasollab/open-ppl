// cont/list.cc
 
#include <stapl/containers/list/list.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
 
#include "conthelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::list<val_tp> list_int_tp;

int prime[20] = { 2,  3,  5,  7, 11, 13, 17, 19, 23, 29,
                 31, 37, 41, 43, 47, 53, 59, 61, 67, 71 };

int fibo[20] = { 1, 2, 3, 4, 8, 13, 21, 34, 55, 89,
               144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946 };


stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container
  list_int_tp l_ct;

  // construct container with non-default initializer
  list_int_tp m_ct(10, 314159);

  // append elements with sequential loop
  for( int i=0; i<20; i++ ) {
    l_ct.push_back(prime[i]);
  }

  // print container size
  stapl::stream<ofstream> zout;
  zout.open("refman_list.txt");
  stapl::do_once( msg_val<int>( zout, "size ", l_ct.size() ) );

  // remove element
  l_ct.erase( l_ct.begin() );

  // append elements with sequential loop
  for( int i=0; i<20; i++ ) {
    l_ct.push_back(fibo[i]);
  }

  // print container size
  stapl::do_once( msg_val<int>( zout, "SIZE ", l_ct.size() ) );

  return EXIT_SUCCESS;
}

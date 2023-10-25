// cont/list.cc

#include <stapl/containers/list/list.hpp>
#include <stapl/views/list_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::list<val_tp> list_int_tp;
typedef stapl::list_view<list_int_tp> list_int_vw_tp;

int prime[20] = { 2,  3,  5,  7, 11, 13, 17, 19, 23, 29,
                 31, 37, 41, 43, 47, 53, 59, 61, 67, 71 };

int fibo[20] = { 1, 2, 3, 4, 8, 13, 21, 34, 55, 89,
               144, 233, 377, 610, 987, 1597, 2584, 4181, 6765, 10946 };


stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container
  list_int_tp l_ct;

  // append elements
  for( int i=0; i<20; i++ ) {
    l_ct.push_back(prime[i]);
  }
  stapl::rmi_fence();

  // construct view over container
  list_int_vw_tp l_vw(l_ct);

  // process elements in parallel
  val_tp max_val;
  max_val = stapl::map_reduce( stapl::identity<val_tp>(),
                               stapl::max<val_tp>(), l_vw );

  // print elements
  stapl::stream<ofstream> zout;
  zout.open("refman_lstvw.txt");
  stapl::do_once( msg_val<val_tp>( zout, "Size ", l_vw.size() ) );
  stapl::serial_io( put_val_wf(zout), l_vw );
  stapl::do_once( msg_val<val_tp>( zout, "Maximum ", max_val ) );

  // append list elements
  for( int i=0; i<20; i++ ) {
    l_ct.push_back(fibo[i]);
  }
  stapl::rmi_fence();

  // need a new view
  list_int_vw_tp n_vw(l_ct);

  // process elements in parallel
  max_val = stapl::map_reduce( stapl::identity<val_tp>(),
                               stapl::max<val_tp>(), n_vw );

  // print elements
  stapl::do_once( msg_val<val_tp>( zout, "Size ", n_vw.size() ) );
  stapl::serial_io( put_val_wf(zout), n_vw );
  stapl::do_once( msg_val<val_tp>( zout, "Maximum ", max_val ) );

  return EXIT_SUCCESS;
}

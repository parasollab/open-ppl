// algo/findif.cc

#include <stapl/runtime.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

#include "algohelp.hpp"
using namespace std;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(40), w_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct);

  // initialize container
  stapl::iota(v_vw, 0);

  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(w_vw, step_wf(base,step));

  // apply algorithm
  int ndx = -1;
  vec_int_vw_tp::reference ref = stapl::find_if(v_vw,
                            bind(stapl::equal_to<val_tp>(),_1,100) );
  if (!stapl::is_null_reference(ref)) {
    ndx = stapl:: index_of(ref);
  }

  int ndx_n = -1;
  vec_int_vw_tp::reference ref_n = stapl::find_if_not(w_vw,
                                  bind(stapl::not_equal_to<val_tp>(),_1,100) );
  if (!stapl::is_null_reference(ref_n)) {
    ndx_n = stapl:: index_of(ref_n);
  }

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_findif.txt");

  stapl::do_once( msg_val<val_tp>( zout, "ndx ", ndx) );
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg_val<val_tp>( zout, "ndx_n ", ndx_n) );
  stapl::serial_io( put_val_wf(zout), w_vw );

  return EXIT_SUCCESS;
}

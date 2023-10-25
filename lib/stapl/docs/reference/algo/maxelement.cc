// algo/maxelement.cc

#include <stapl/runtime.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "algohelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

typedef stapl::less<val_tp> less_fn;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(40), w_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct);

  // initialize container
  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));
 
  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(w_vw, repeat_wf(base,rep));

  // apply algorithm
  int ndx = -1;
  vec_int_vw_tp::reference ref = stapl::max_element(v_vw);
  if( !stapl::is_null_reference(ref) ) {
    ndx = stapl::index_of(ref);
  }

  int ndx_c = -1;
  vec_int_vw_tp::reference ref_c = stapl::max_element(w_vw, less_fn());
  if( !stapl::is_null_reference(ref_c) ) {
    ndx_c = stapl::index_of(ref_c);
  }

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_maxelement.txt");
  stapl::do_once( msg_val<int>( zout, "ndx", ndx ) );
  stapl::do_once( msg_val<int>( zout, "ndx_c", ndx_c ) );
  stapl::serial_io( put_val_wf(zout), v_vw );

  return EXIT_SUCCESS;
}

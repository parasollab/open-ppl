// algo/binsrch.cc

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

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(40), w_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct);

  // initialize container
  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  base = 1;
  step = 2;
  stapl::generate(w_vw, step_wf(base,step));

  // apply algorithm
  val_tp val;
  val = v_vw[ v_vw.size()*.75 ];
  bool v_test = stapl::binary_search(v_vw, val, stapl::less<val_tp>() );
  val = v_vw[ v_vw.size()*.25 ];
  bool w_test = stapl::binary_search(w_vw, val);

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_binsrch.txt");

  stapl::do_once( msg_val<bool>( zout, "v_vw test: ", v_test ) );
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg_val<bool>( zout, "w_vw test: ", w_test ) );
  stapl::serial_io( put_val_wf(zout), w_vw );

  return EXIT_SUCCESS;
}

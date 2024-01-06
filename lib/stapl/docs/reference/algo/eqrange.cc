// algo/eqrange.cc

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
  vec_int_tp v_ct(40), w_ct(40), u_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct), u_vw(u_ct);

  // initialize container
  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(v_vw, repeat_wf(base,rep));

  val_tp v_val;
  val_tp w_val;

  // apply algorithm
  vec_int_vw_tp x_vw = stapl::equal_range( v_vw, v_val, stapl::less<val_tp>() );
  vec_int_vw_tp y_vw = stapl::equal_range( u_vw, w_val );

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_eqrange.txt");

  stapl::do_once( msg( zout, "v_vw" ) );
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg( zout, "w_vw" ) );
  stapl::serial_io( put_val_wf(zout), w_vw );

  stapl::do_once( msg( zout, "x_vw" ) );
  stapl::serial_io( put_val_wf(zout), x_vw );
  stapl::do_once( msg( zout, "y_vw" ) );
  stapl::serial_io( put_val_wf(zout), y_vw );

  return EXIT_SUCCESS;
}

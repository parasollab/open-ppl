// algo/adjdiff.cc

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
  vec_int_tp v_ct(40), w_ct(40), x_ct(40), y_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct), x_vw(x_ct), y_vw(y_ct);

  // initialize container
  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(v_vw, repeat_wf(base,rep));

  // apply algorithm
  typedef stapl::minus<val_tp> oper;
  stapl::adjacent_difference(v_vw, x_vw, oper());
  stapl::adjacent_difference(w_vw, y_vw);

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_adjman.txt");

  stapl::do_once( msg( zout, "x_vw" ) );
  stapl::serial_io( put_val_wf(zout), x_vw );

  stapl::do_once( msg( zout, "y_vw" ) );
  stapl::serial_io( put_val_wf(zout), y_vw );

  return EXIT_SUCCESS;
}

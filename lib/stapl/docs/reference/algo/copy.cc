// algo/copy.cc

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

  // construct containers and views
  vec_int_tp v_ct(40), w_ct(40), x_ct(40), y_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct), x_vw(x_ct), y_vw(y_ct);

  // initialize containers
  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(w_vw, repeat_wf(base,rep));

  stapl::generate(x_vw, stapl::random_sequence());
  stapl::iota(y_vw, 0);

  // apply algorithms
  stapl::copy( v_vw, x_vw );
  stapl::copy_n( w_vw, y_vw, w_vw.size()/2 );

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_copy.txt");
  stapl::do_once( msg( zout, "x_vw " ) );
  stapl::serial_io( put_val_wf(zout), x_vw );
  stapl::do_once( msg( zout, "y_vw " ) );
  stapl::serial_io( put_val_wf(zout), y_vw );

  return EXIT_SUCCESS;
}

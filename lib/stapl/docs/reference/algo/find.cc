// algo/find.cc

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
  vec_int_tp v_ct(40);
  vec_int_vw_tp v_vw(v_ct);

  // initialize container
  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  // apply algorithm
  const val_tp val = v_vw[v_vw.size()/2];
  int ndx = -1;

  vec_int_vw_tp::reference ref = stapl::find(v_vw, val);
  if( !stapl::is_null_reference(ref) ) {
    ndx = stapl::index_of(ref);
  }

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_find.txt");

  stapl::do_once( msg_val<val_tp>( zout, "ndx ", ndx ) );
  stapl::serial_io( put_val_wf(zout), v_vw );

  return EXIT_SUCCESS;
}

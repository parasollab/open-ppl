// algo/findfirst.cc

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
  vec_int_tp v_ct(40), w_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct);

  // initialize containers
  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(w_vw, repeat_wf(base,rep));

  // apply algorithm
  int ndx = -1;
  vec_int_vw_tp::reference ref = stapl::find_first_of(v_vw, w_vw);
  if( !stapl::is_null_reference(ref) ) {
    ndx = stapl::index_of(ref);
  }

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_find_first_of.txt");
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg( zout, " " ) );
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg_val<val_tp>( zout, "Result ", ndx ) );

  return EXIT_SUCCESS;
}

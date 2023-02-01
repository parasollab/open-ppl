// algo/minvalue.cc

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

  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(w_vw, repeat_wf(base,rep));

  // apply algorithm
  typedef stapl::less<val_tp> less_fn;
  val_tp v_val = stapl::min_value(v_vw, less_fn());
  val_tp w_val = stapl::min_value(w_vw);

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_minval.txt");
  stapl::do_once( msg_val<val_tp>( zout, "v_val ", v_val ) );
  stapl::do_once( msg_val<val_tp>( zout, "w_val ", w_val ) );

  return EXIT_SUCCESS;
}

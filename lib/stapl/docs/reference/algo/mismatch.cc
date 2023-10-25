// algo/mismatch.cc

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

typedef std::pair<vec_int_vw_tp::reference,vec_int_vw_tp::reference>
        pair_ref_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(40), w_ct(40), x_ct(40), y_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct), x_vw(x_ct), y_vw(y_ct);

  // initialize containers
  stapl::iota(v_vw, 0);
  stapl::iota(w_vw, 0);

  int base = 0;
  int step = 5;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(x_vw, step_wf(base,step));

  base = 0;
  step = 1;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(y_vw, step_wf(base,step));

  // apply algorithm
  int ndx_v = -1, ndx_w = -1;
  typedef stapl::equal_to<val_tp> eq_wf;
  pair_ref_tp ref_pair_pr = stapl::mismatch(v_vw, x_vw, eq_wf() );
  if( !stapl::is_null_reference(ref_pair_pr.first) ) {
    ndx_v = stapl::index_of(ref_pair_pr.first);
  }
  if( !stapl::is_null_reference(ref_pair_pr.second) ) {
    ndx_w = stapl::index_of(ref_pair_pr.second);
  }

  int ndx_x = -1, ndx_y = -1;
  pair_ref_tp ref_p = stapl::mismatch(w_vw, y_vw );
  if( !stapl::is_null_reference(ref_p.first) ) {
    ndx_x = stapl::index_of(ref_p.first);
  }
  if( !stapl::is_null_reference(ref_p.second) ) {
    ndx_y = stapl::index_of(ref_p.second);
  }

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_mismatch.txt");

  stapl::do_once( msg_val<int>( zout, "v_ndx ", ndx_v ) );
  stapl::do_once( msg_val<int>( zout, "w_ndx ", ndx_w ) );
  stapl::do_once( msg_val<int>( zout, "x_ndx ", ndx_x ) );
  stapl::do_once( msg_val<int>( zout, "y_ndx ", ndx_y ) );

  return EXIT_SUCCESS;
}

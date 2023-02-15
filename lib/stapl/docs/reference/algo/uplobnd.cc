// algo/uplobnd.cc

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

val_tp data1[] = {
};
val_tp data2[] = {
};
val_tp data3[] = {
};
val_tp data4[] = {
};

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(40), w_ct(40), x_ct(40), y_ct(40);

  // initialize container

// FIXME

  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct), x_vw(x_ct), y_vw(y_ct);;

  // apply algorithms
  val_tp val;

  int v_ndx = -1;
  vec_int_vw_tp::reference v_ref = stapl::lower_bound( v_vw, val,
                                   stapl::less<val_tp>() );
  if( !stapl::is_null_reference(v_ref) ) {
    v_ndx = stapl::index_of(v_ref);
  }

  int w_ndx = -1;
  vec_int_vw_tp::reference w_ref = stapl::lower_bound( w_vw, val );
  if( !stapl::is_null_reference(w_ref) ) {
    w_ndx = stapl::index_of(w_ref);
  }

  int x_ndx = -1;
  vec_int_vw_tp::reference x_ref = stapl::upper_bound( x_vw, val,
                                   stapl::less<val_tp>() );
  if( !stapl::is_null_reference(x_ref) ) {
    x_ndx = stapl::index_of(x_ref);
  }

  int y_ndx = -1;
  vec_int_vw_tp::reference y_ref = stapl::upper_bound( y_vw, val );
  if( !stapl::is_null_reference(y_ref) ) {
    y_ndx = stapl::index_of(y_ref);
  }

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_uplobnd.txt");

  stapl::do_once( msg_val<int>( zout, "v_ndx", v_ndx ) );
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg_val<int>( zout, "w_ndx", w_ndx ) );
  stapl::serial_io( put_val_wf(zout), w_vw );

  stapl::do_once( msg_val<int>( zout, "x_ndx", x_ndx ) );
  stapl::serial_io( put_val_wf(zout), x_vw );
  stapl::do_once( msg_val<int>( zout, "y_ndx", y_ndx ) );
  stapl::serial_io( put_val_wf(zout), y_vw );

  return EXIT_SUCCESS;
}

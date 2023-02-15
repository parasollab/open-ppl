/* ****************************** NOT DONE ****************************** */
// these algorithms are broken - unit tests are turned off
// this example gets the same error messages as when they are compiled

// algo/permutation.cc

#include <stapl/runtime.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "algohelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

typedef stapl::array<val_tp> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;

template <typename T>
struct less_pred
{
  typedef T    argument_type;
  typedef bool result_type;
  result_type operator() (const T& x, const T& y) const
  {
    return x < y;
  }
};

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
  ary_int_tp v_ct(40), w_ct(4), x_ct(40), y_ct(40);

  // initialize containers
  for( int i=0; i<20; i++ ) {
    v_ct[i] = data1[i];
  }
  for( int i=0; i<20; i++ ) {
    w_ct[i] = data2[i];
  }
  for( int i=0; i<20; i++ ) {
    x_ct[i] = data3[i];
  }
  for( int i=0; i<20; i++ ) {
    y_ct[i] = data4[i];
  }
  stapl::rmi_fence();

  ary_int_vw_tp v_vw(v_ct), w_vw(w_ct), x_vw(x_ct), y_vw(y_ct);

  // apply algorithms
#if 0
  bool w_test = stapl::next_permutation(w_vw);
  bool y_test = stapl::prev_permutation(y_vw);

  bool u_test = stapl::next_permutation(v_vw, less_pred<val_tp>() );
  bool x_test = stapl::prev_permutation(x_vw, less_pred<val_tp>() );
#endif

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_permutation.txt");

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
/* ****************************** NOT DONE ****************************** */

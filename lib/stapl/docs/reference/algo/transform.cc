// algo/transform.cc

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

typedef stapl::negate<val_tp> neg_wf;
typedef stapl::min<val_tp> min_wf;
typedef stapl::max<val_tp> max_wf;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(40), w_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct);

  vec_int_tp x_ct(40), y_ct(40), z_ct(40);
  vec_int_vw_tp x_vw(x_ct), y_vw(y_ct), z_vw(z_ct);

  // initialize container
  stapl::iota(v_vw, 0);

  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(x_vw, step_wf(base,step));

  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(y_vw, repeat_wf(base,rep));

  // apply algorithm
  stapl::transform(v_vw, w_vw, neg_wf());
  stapl::transform(x_vw, y_vw, z_vw, max_wf());

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_transform.txt");
  stapl::do_once( msg( zout, "v_vw " ) );
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg( zout, "z_vw " ) );
  stapl::serial_io( put_val_wf(zout), z_vw );

  return EXIT_SUCCESS;
}

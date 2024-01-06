// algo/radsort.cc

#include <stapl/runtime.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/algorithms/sorting.hpp>

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
  stapl::generate(v_vw, stapl::random_sequence());

  // apply algorithm
  stapl::radix_sort(v_vw);

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_radsort.txt");
  stapl::do_once( msg( zout, "v_vw" ) );
  stapl::serial_io( put_val_wf(zout), v_vw );

  return EXIT_SUCCESS;
}

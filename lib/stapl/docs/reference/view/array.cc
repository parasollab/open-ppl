// cont/array.cc

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::array<val_tp> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;


stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  ary_int_tp a_ct(40);
  ary_int_vw_tp a_vw(a_ct);

  // initialize container in parallel
  stapl::iota(a_vw, 0);

  // process elements in parallel
  val_tp sum = stapl::map_reduce( stapl::identity<val_tp>(),
                                  stapl::plus<val_tp>(), a_vw );

  // print elements
  stapl::stream<ofstream> zout;
  zout.open("refman_aryvw.txt");

  stapl::serial_io( put_val_wf(zout), a_vw );
  stapl::do_once( msg_val<val_tp>( zout, "Sum ", sum ) );

  return EXIT_SUCCESS;
}

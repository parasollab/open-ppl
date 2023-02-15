// view/repeat.cc

#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

typedef stapl::plus<val_tp> add_int_wf;
typedef stapl::multiplies<val_tp> mul_int_wf;


stapl::exit_code stapl_main(int argc, char **argv) {

  size_t sz = 0;
  // construct container
  vec_int_tp a_ct(sz), b_ct(sz);

  // construct view over container
  vec_int_vw_tp a_vw(a_ct), b_vw(b_ct);

  // initialize container
 
  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));

  val_tp scalar = 10;

  // process elements through view
  val_tp sum = stapl::map_reduce( mul_int_wf(), add_int_wf(),
                                  stapl::make_repeat_view(scalar), a_vw);

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_rptvw.txt");
  stapl::serial_io( put_val_wf(zout), a_vw );
  stapl::do_once( msg_val<val_tp>( zout, "Sum ", sum ) );

  return EXIT_SUCCESS;
}

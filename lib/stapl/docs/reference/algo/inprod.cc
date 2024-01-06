// algo/inprod.cc

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
  vec_int_tp v_ct(40), w_ct(40), u_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct), u_vw(u_ct);

  // initialize container
  stapl::iota(v_vw, 0);

  int base = 0;
  int step = 2;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(w_vw, step_wf(base,step));

  base = 1;
  step = 2;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(u_vw, step_wf(base,step));

  // apply algorithm
  val_tp init = 0;
  val_tp prod_v_w = stapl::inner_product( v_vw, w_vw, init );
  val_tp prod_v_u = stapl::inner_product( v_vw, u_vw, init,
                                          stapl::plus<val_tp>(),
                                          stapl::max<val_tp>() );

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_inprod.txt");

  stapl::do_once( msg_val<val_tp>( zout, "prod_v_w ", prod_v_w ) );
  stapl::do_once( msg_val<val_tp>( zout, "prod_v_u ", prod_v_u ) );

  return EXIT_SUCCESS;
}

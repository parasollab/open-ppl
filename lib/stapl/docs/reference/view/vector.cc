// cont/vector.cc

#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;


stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(30);
  vec_int_vw_tp v_vw(v_ct);
  val_tp max_val;

  // initialize container in parallel
  int base = 10;
  int step = 2;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  // process container elements parallel
  max_val = stapl::map_reduce( stapl::identity<val_tp>(),
                               stapl::max<val_tp>(), v_vw );

  // print elements
  stapl::stream<ofstream> zout;
  zout.open("refman_vecvw.txt");
  stapl::do_once( msg_val<val_tp>( zout, "Size ", v_vw.size() ) );
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg_val<val_tp>( zout, "Maximum ", max_val ) );

  // append elements
  v_ct.push_back(1001);
  v_ct.push_back(2002);
  v_ct.push_back(3003);
  v_ct.push_back(4004);
  stapl::rmi_fence();

  // need a new view
  vec_int_vw_tp w_vw(v_ct);

  // process elements in parallel
  max_val = stapl::map_reduce( stapl::identity<val_tp>(),
                               stapl::max<val_tp>(), w_vw );

  // print elements
  stapl::do_once( msg_val<val_tp>( zout, "Size ", w_vw.size() ) );
  stapl::serial_io( put_val_wf(zout), w_vw );
  stapl::do_once( msg_val<val_tp>( zout, "Maximum ", max_val ) );

  return EXIT_SUCCESS;
}

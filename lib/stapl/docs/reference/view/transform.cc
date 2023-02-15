// view/transform.cc

#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/views/transform_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

template <typename T>
struct divby2_wf {
  typedef T result_type;
  template<typename Ref>
  result_type operator()(Ref x) {
    return x / 2;
  }
};

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

typedef stapl::identity<val_tp> id_int_wf;
typedef stapl::plus<val_tp> add_int_wf;
typedef divby2_wf<val_tp> divby2_int_wf;

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container
  vec_int_tp v_ct(30);
  vec_int_vw_tp const v_vw(v_ct);
  val_tp val;

  // initialize container in parallel
  int base = 10;
  int step = 2;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  // construct view over container
  typedef typename stapl::transform_view<vec_int_vw_tp, divby2_int_wf> tf_vw_tp;
  tf_vw_tp tf_vw(v_vw, divby2_int_wf());

  // process container elements in parallel
  val = stapl::map_reduce( id_int_wf(), add_int_wf(),
        stapl::transform_view<vec_int_vw_tp, divby2_int_wf>
               ( v_vw, divby2_int_wf() ));

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_xfrmvw.txt");
  stapl::do_once( msg_val<val_tp>( zout, "Size ", v_vw.size() ) );
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg_val<val_tp>( zout, "Sum ", val ) );

  return EXIT_SUCCESS;
}

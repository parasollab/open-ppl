#include <iostream>
#include <fstream>
#include "ch5.hpp"

#include <stapl/vector.hpp>
#include <stapl/views/strided_view.hpp>

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

typedef stapl::strided_view<vec_int_vw_tp>::type str_vec_int_vw_tp; // ## 1

int ex_502(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_502.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 502" ) );
  int result = ex_502(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

struct add_wf {
  typedef void result_type;
  template<typename Ref1, typename Ref2, typename Ref3>
  result_type operator()(Ref1 x, Ref2 y, Ref3 z) {
    z = x + y;
  }
};

int ex_502(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  vec_int_tp a_ct(sz), b_ct(sz*2), c_ct(sz*4);
  vec_int_vw_tp a_vec_vw(a_ct), b_vec_vw(b_ct), c_vec_vw(c_ct);

  int stride = 2;
  str_vec_int_vw_tp a_str_vw = stapl::make_strided_view(a_vec_vw, stride, 0); // ## 2

  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vec_vw, step_wf(base,step));

  stapl::do_once( msg( zout, "a_vec:" ) );
  stapl::serial_io(put_val_wf(zout), a_vec_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::do_once( msg( zout, "a_str:" ) );
  stapl::serial_io(put_val_wf(zout), a_str_vw );
  stapl::do_once( msg(zout, "\n" ) );

  int rep = 49;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(b_vec_vw, repeat_wf(base,rep));

  stride = 2; // ## 3
  str_vec_int_vw_tp b_str_vw = stapl::make_strided_view(b_vec_vw, stride, 0);

  stapl::do_once( msg( zout, "b_vec:" ) );
  stapl::serial_io(put_val_wf(zout), b_vec_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::do_once( msg( zout, "b_str:" ) );
  stapl::serial_io(put_val_wf(zout), b_str_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stride = 4; // ## 4
  str_vec_int_vw_tp c_str_vw = stapl::make_strided_view(c_vec_vw, stride, 0);

  stapl::map_func(add_wf(), a_vec_vw, b_str_vw, c_str_vw); // ## 5

  stapl::do_once( msg( zout, "c_vec:" ) );
  stapl::serial_io(put_val_wf(zout), c_vec_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::do_once( msg( zout, "c_str:" ) );
  stapl::serial_io(put_val_wf(zout), c_str_vw );
  stapl::do_once( msg(zout, "\n" ) );

  int val = c_ct[sz*4-1];
  stapl::rmi_fence();
  return val;
}

#include <iostream>
#include <fstream>

#include <stapl/utility/do_once.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include <stapl/skeletons/serial.hpp>

using namespace std;
#include "ch2_msg.hpp"
#include "ch2_put.hpp"

typedef stapl::array<int> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;
typedef stapl::identity<int> id_int_wf;
typedef stapl::negate<int> neg_int_wf;
typedef stapl::minus<int> sub_int_wf;
typedef stapl::plus<int> add_int_wf;

int ex_208(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_208.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 208" ) );
  int result = ex_208(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

int ex_208(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  ary_int_tp a(sz), b(sz), c(sz);
  ary_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step)); // ## 1
  base = 0;
  step = 5;
  stapl::generate(b_vw, step_wf(base,step)); // ## 2

  stapl::do_once( msg( zout, "a:") );
  stapl::serial_io(put_val_wf(zout), a_vw);
  stapl::do_once( msg(zout, "\n" ) );

  stapl::do_once( msg( zout, "b:") );
  stapl::serial_io(put_val_wf(zout), b_vw);
  stapl::do_once( msg(zout, "\n" ) );

  stapl::transform(a_vw, b_vw, c_vw, add_int_wf()); // ## 3

  stapl::do_once( msg( zout, "c:") );
  stapl::serial_io(put_val_wf(zout), c_vw);
  stapl::do_once( msg(zout, "\n" ) );

  int res = c[sz-1];
  stapl::rmi_fence();
  return res;
}

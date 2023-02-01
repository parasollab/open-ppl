#include <iostream>
#include <fstream>
#include <stapl/utility/do_once.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp> // ## 1
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include <stapl/skeletons/serial.hpp>

using namespace std;
#include "ch2_msg.hpp"
#include "ch2_put.hpp" // ## 2

typedef stapl::array<int> ary_int_tp; // ## 3
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp; // ## 4

int ex_202(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_202.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 202" ) );
  int result = ex_202(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

int ex_202(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  ary_int_tp a(sz); // ## 5
  ary_int_vw_tp a_vw(a); // ## 6

  int base = 10;
  int step = 2;
  typedef stapl::sequence<int> step_wf;  // ## 7
  stapl::generate(a_vw, step_wf(base,step)); // ## 8

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(put_val_wf(zout), a_vw); // ## 9
  stapl::do_once( msg(zout, "\n" ) );

  return 1;
}

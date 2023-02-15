#include <iostream>
#include <fstream>

#include <stapl/utility/do_once.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/runtime.hpp>
#include <stapl/stream.hpp>
#include <stapl/skeletons/serial.hpp>
#include <stapl/stream.hpp>

using namespace std;
#include "ch2_msg.hpp"
#include "ch2_ndx.hpp"
#include "ch2_get.hpp"

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

int ex_209(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  zin.open("primes_1000.txt");
  stapl::stream<ofstream> zout;
  zout.open("ex_209.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 209" ) );
  int result = ex_209(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

int ex_209(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  vec_int_tp a(sz), b(sz), c(sz);
  vec_int_vw_tp a_vw(a), b_vw(b), c_vw(c);

  stapl::do_once( msg( zout, "a:") );
  stapl::serial_io(get_val_wf(zin), a_vw); // ## 1
  stapl::do_once( msg(zout, "\n" ) );

  stapl::map_func(stapl::assign<vec_int_vw_tp::value_type>(), // ## 2
                  a_vw, b_vw);

  stapl::do_once( msg( zout, "b:") );
  stapl::serial_io(put_ndx_val_wf(zout),
                   stapl::counting_view<int>(sz), b_vw); // ## 3
  stapl::do_once( msg(zout, "\n" ) );

  stapl::copy( a_vw, c_vw); // ## 4

  stapl::do_once( msg( zout, "c:") );
  stapl::serial_io(put_ndx_val_wf(zout),
                   stapl::counting_view<int>(c.size()), vec_int_vw_tp(c));
  stapl::do_once( msg(zout, "\n" ) );

  int res = a[sz-1];
  stapl::rmi_fence();
  return res;
}

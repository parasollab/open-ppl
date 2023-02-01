#include <iostream>
#include <fstream>

#include <stapl/utility/do_once.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include <stapl/stream.hpp>
#include <stapl/skeletons/serial.hpp>
#include <stapl/stream.hpp>

using namespace std;
#include "ch2_msg.hpp"
#include "ch2_put.hpp"
#include "ch2_get.hpp"

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

typedef stapl::array<int> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;

int ex_212(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  zin.open("primes_1000.txt");
  stapl::stream<ofstream> zout;
  zout.open("ex_212.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 212" ) );
  int result = ex_212(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

struct unary_wf { // ## 1
  typedef void result_type;
  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 x, Ref2 z) { // ## 2
    z = 1 + (x % 10); // ## 3
  }
};

struct binary_wf {
  typedef void result_type;
  template <typename Ref1, typename Ref2, typename Ref3> // ## 4
  result_type operator()(Ref1 x, Ref2 y, Ref3 z) {
    z = (x % 10) + (y % 10);
  }
};

struct unary_val_wf {
  typedef int result_type; // ## 5
  template <typename Ref1>
  result_type operator()(Ref1 x) {
    return 1 + (x % 10);
  }
};

int ex_212(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  vec_int_tp a(sz), b(sz);
  vec_int_vw_tp a_vw(a), b_vw(b);

  ary_int_tp c(sz), d(sz), e(sz);
  ary_int_vw_tp c_vw(c), d_vw(d), e_vw(e);

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::map_func( unary_wf(), a_vw, b_vw ); // ## 6

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(put_val_wf(zout), a_vw);
  stapl::do_once( msg(zout, "\n" ) );

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::transform( a_vw, b_vw, unary_val_wf() ); // ## 7

  stapl::do_once( msg( zout, "b:" ) );
  stapl::serial_io(put_val_wf(zout), b_vw);
  stapl::do_once( msg(zout, "\n" ) );

  stapl::serial_io(get_val_wf(zin), c_vw);
  stapl::serial_io(get_val_wf(zin), d_vw);
  stapl::map_func( binary_wf(), c_vw, d_vw, e_vw );

  stapl::do_once( msg( zout, "e:" ) );
  stapl::serial_io(put_val_wf(zout), e_vw);
  stapl::do_once( msg(zout, "\n" ) );

  int res = a[ sz-1 ];
  stapl::rmi_fence();
  return res;
}

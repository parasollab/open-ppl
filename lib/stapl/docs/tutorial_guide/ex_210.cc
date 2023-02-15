#include <iostream>
#include <fstream>

#include <stapl/utility/do_once.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/runtime.hpp>
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

typedef stapl::identity<int> id_int_wf;
typedef stapl::negate<int> neg_int_wf;
typedef stapl::minus<int> sub_int_wf;
typedef stapl::max<int> max_int_wf;
typedef stapl::min<int> min_int_wf;
typedef stapl::plus<int> add_int_wf;

int ex_210(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  zin.open("primes_1000.txt");
  stapl::stream<ofstream> zout;
  zout.open("ex_210.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 210" ) );
  int result = ex_210(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

int ex_210(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  ary_int_tp a(sz), b(sz);
  ary_int_vw_tp a_vw(a), b_vw(b);
  vec_int_tp c(sz), d(sz);
  vec_int_vw_tp c_vw(c), d_vw(d);
  int min_val, max_val;

  stapl::serial_io(get_val_wf(zin), a_vw);
  stapl::serial_io(get_val_wf(zin), b_vw);
  stapl::serial_io(get_val_wf(zin), c_vw);
  stapl::serial_io(get_val_wf(zin), d_vw);

  min_val = stapl::map_reduce( neg_int_wf(), min_int_wf(), a_vw); // ## 1

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(put_val_wf(zout), a_vw);
  stapl::do_once( msg(zout, "\n" ) );
  stapl::do_once( msg_val<int>( zout, "min:", min_val ) );

  max_val = stapl::map_reduce( neg_int_wf(), max_int_wf(), c_vw); // ## 2

  stapl::do_once( msg( zout, "c:" ) );
  stapl::serial_io(put_val_wf(zout), c_vw);
  stapl::do_once( msg(zout, "\n" ) );
  stapl::do_once( msg_val<int>( zout, "max:", max_val ) );

  stapl::scan(a_vw, b_vw, min_int_wf(), false); // ## 3
  min_val = b_vw[ b_vw.size() - 1 ];

  stapl::do_once( msg( zout, "b:" ) );
  stapl::serial_io(put_val_wf(zout), b_vw);
  stapl::do_once( msg(zout, "\n" ) );
  stapl::do_once( msg_val<int>( zout, "min:", min_val ) );

  stapl::scan(c_vw, d_vw, max_int_wf(), false); // ## 4
  max_val = d_vw[ d_vw.size() - 1 ];

  stapl::do_once( msg( zout, "d:" ) );
  stapl::serial_io(put_val_wf(zout), d_vw);
  stapl::do_once( msg(zout, "\n" ) );
  stapl::do_once( msg_val<int>( zout, "max:", max_val ) );

  return max_val;
}

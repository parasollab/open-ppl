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
typedef stapl::max<int> max_int_wf;
typedef stapl::min<int> min_int_wf;

int ex_205(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_205.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 205" ) );
  int result = ex_205(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

int ex_205(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  ary_int_tp a(sz);
  ary_int_vw_tp a_vw(a);

  stapl::generate( a_vw, stapl::random_sequence(1000)); // ## 1

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(put_val_wf(zout), a_vw);
  stapl::do_once( msg(zout, "\n" ) );

  int max = stapl::map_reduce( id_int_wf(), max_int_wf(), a_vw); // ## 2

  return max;
}

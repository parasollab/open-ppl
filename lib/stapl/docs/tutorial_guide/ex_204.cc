#include <iostream>
#include <fstream>

#include <stapl/utility/do_once.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include <stapl/skeletons/serial.hpp>

using namespace std;
#include "ch2_msg.hpp"
#include "ch2_put.hpp"

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;
typedef stapl::identity<int> id_int_wf; // ## 1
typedef stapl::negate<int> neg_int_wf;
typedef stapl::minus<int> sub_int_wf;
typedef stapl::plus<int> add_int_wf;

int ex_204(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_204.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 204" ) );
  int result = ex_204(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

int ex_204(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  vec_int_tp a(sz);
  vec_int_vw_tp a_vw(a);

  int rep = 42;
  int base = 0;
  typedef stapl::block_sequence<int> repeat_wf; // ## 2
  stapl::generate(a_vw, repeat_wf(base,rep)); // ## 3

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(put_val_wf(zout), a_vw);
  stapl::do_once( msg(zout, "\n" ) );

  int tot = stapl::map_reduce( id_int_wf(), add_int_wf(), a_vw); // ## 4

  int sum = stapl::reduce( a_vw, add_int_wf() ); // ## 5

  return tot + sum;
}

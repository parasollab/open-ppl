#include <iostream>
#include <fstream>

#include <stapl/utility/do_once.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/views/map_view.hpp>
#include <stapl/containers/map/map.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/views/counting_view.hpp>
#include <stapl/views/repeated_view.hpp>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include <stapl/skeletons/serial.hpp>

using namespace std;
#include "ch2_msg.hpp"
#include "ch2_put.hpp" // ## 1

typedef stapl::map<int,int> map_int_tp;
typedef stapl::map_view<map_int_tp> map_int_vw_tp;
typedef stapl::indexed_domain<int> ndx_dom_tp;

typedef stapl::identity<int> id_int_wf;
typedef stapl::negate<int> neg_int_wf;
typedef stapl::minus<int> sub_int_wf;
typedef stapl::plus<int> add_int_wf;

extern int prime_nums[], rand_nums[];

struct map_int_val_wf {
  typedef int result_type;
  template<typename Pair>
  result_type operator()(Pair const &pair) {
    return pair.second; // ## 2
  }
};

int ex_206(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_206.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 206" ) );
  int result = ex_206(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

int ex_206(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  ndx_dom_tp c_dom(0, prime_nums[sz-1]);
  map_int_tp c(c_dom);
  map_int_vw_tp c_vw(c);

  for ( size_t i = 0; i < sz; i++ ) {
    c_vw[ prime_nums[i] ] = rand_nums[i]; // ## 3
  }

  int tot = stapl::map_reduce( map_int_val_wf(), add_int_wf(), c_vw); // ## 4

  stapl::do_once( msg( zout, "c:" ) );
  stapl::serial_io(put_map_val_wf(zout), c_vw); // ## 5
  stapl::do_once( msg(zout, "\n" ) );

  return tot;
}

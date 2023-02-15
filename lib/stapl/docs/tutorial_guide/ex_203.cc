#include <iostream>
#include <fstream>
#include <stapl/utility/do_once.hpp>
#include <stapl/domains/indexed.hpp>
#include <stapl/views/map_view.hpp> // ## 1
#include <stapl/containers/map/map.hpp> // ## 2
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/stream.hpp>
#include <stapl/runtime.hpp>
#include <stapl/skeletons/serial.hpp>

using namespace std;
#include "ch2_msg.hpp"

extern int prime_nums[], rand_nums[]; // ## 3

typedef stapl::map<int,int> map_int_tp; // ## 4
typedef stapl::map_view<map_int_tp> map_int_vw_tp;
typedef stapl::indexed_domain<int> ndx_dom_tp; // ## 5

class put_map_val_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  put_map_val_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Ref>
  result_type operator()(Ref pair) {
    typename Ref::first_reference left = pair.first;
    typename Ref::second_reference right = pair.second;
    m_zout << "{" << left << "}= " << right << "\n"; // ## 6
  }

  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

int ex_203(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_203.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 203" ) );
  int result = ex_203(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

int ex_203(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  ndx_dom_tp c_dom(0, prime_nums[sz-1]); // ## 7
  map_int_tp c(c_dom); // ## 8
  map_int_vw_tp c_vw(c); // ## 9

  for ( size_t i = 0; i < sz; i++ ) {
    c_vw[ prime_nums[i] ] = rand_nums[i]; // ## 10
  }

  stapl::do_once( msg( zout, "c:" ) );
  stapl::serial_io(put_map_val_wf(zout), c_vw);  // ## 11
  stapl::do_once( msg(zout, "\n" ) );

  int res = c_vw[ prime_nums[sz-1] ];
  stapl::rmi_fence(); // ## 12
  return res;
}

#include <iostream>
#include <fstream>
#include <stapl/utility/do_once.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/containers/vector/vector.hpp> // ## 1
#include <stapl/algorithms/algorithm.hpp> // ## 2
#include <stapl/algorithms/functional.hpp>
#include <stapl/stream.hpp> // ## 3
#include <stapl/runtime.hpp>
#include <stapl/skeletons/serial.hpp>

using namespace std;
#include "ch2_msg.hpp" // ## 4

typedef stapl::vector<int> vec_int_tp;// ## 5
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp; // ## 6

class put_val_wf // ## 7
{
private:
  stapl::stream<ofstream> m_zout;
public:
  put_val_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Ref> // ## 8
  result_type operator()(Ref val) {
    m_zout << val << " ";
  }

  void define_type(stapl::typer& t) { // ## 9
    t.member(m_zout);
  }
};

int ex_201(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_201.out");

  size_t size = 100;
  stapl::do_once( msg(zout, "Example 201" ) );
  int result = ex_201(size, zin, zout);
  stapl::do_once( msg_val<int>(zout, "Result ", result ) );

  return EXIT_SUCCESS;
}

int ex_201(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  vec_int_tp a_ct(sz); // ## 10
  vec_int_vw_tp a_vw(a_ct); // ## 11

  stapl::iota(a_vw, 0); // ## 12

  stapl::do_once( msg(zout, "a:" ) );
  stapl::serial_io(put_val_wf(zout), a_vw); // ## 13
  stapl::do_once( msg(zout, "\n" ) );

  return 1;
}

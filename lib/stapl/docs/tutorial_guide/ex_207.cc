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
typedef stapl::identity<int> id_int_wf;
typedef stapl::negate<int> neg_int_wf;
typedef stapl::minus<int> sub_int_wf;
typedef stapl::plus<int> add_int_wf;

class put_ndx_val_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  put_ndx_val_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename Ref1, typename Ref2> // ## 1
  result_type operator()(Ref1 pos, Ref2 val) {  // ## 2
    m_zout << "[" << pos << "]= " << val <<  "\n";
  }

  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

int ex_207(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_207.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 207" ) );
  int result = ex_207(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

int ex_207(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  vec_int_tp a(sz), b(sz);
  vec_int_vw_tp a_vw(a), b_vw(b);

  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(put_ndx_val_wf(zout),
                  stapl::counting_view<int>(sz), a_vw ); // ## 3
  stapl::do_once( msg(zout, "\n" ) );

  stapl::transform(a_vw, b_vw, neg_int_wf()); // ## 4

  stapl::do_once( msg( zout, "b:" ) );
  stapl::serial_io(put_ndx_val_wf(zout),
                  stapl::counting_view<int>(sz), b_vw );
  stapl::do_once( msg(zout, "\n" ) );

  int res = b[sz-1];
  stapl::rmi_fence();
  return res;
}

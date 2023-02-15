#include <iostream>
#include <fstream>
#include "ch5.hpp"

#include <stapl/vector.hpp>
#include <stapl/views/zip_view.hpp>

typedef stapl::vector<int>               vec_int_tp;
typedef stapl::vector_view<vec_int_tp>   vec_int_vw_tp;

typedef stapl::zip_view<vec_int_vw_tp, vec_int_vw_tp, vec_int_vw_tp> 
                        zip3_vec_int_vw_tp; // ## 1

extern int prime_nums[], rand_nums[];

int ex_508(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_508.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 508" ) );
  int result = ex_508(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

class put_triple_wf
{
private:
  stapl::stream<ofstream> m_zout;
public:
  put_triple_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }
  typedef void result_type;
  template <typename Arg>
  result_type operator()(Arg arg) {
    tuple<int,int,int> tup = arg; // ## 2
    m_zout << get<0>(tup) << "," << get<1>(tup) << "," << get<2>(tup) << endl; // ## 3
  }
  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

int ex_508(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  vec_int_tp a_ct(sz), b_ct(sz), c_ct(sz);
  vec_int_vw_tp a_vw(a_ct), b_vw(b_ct), c_vw(c_ct);

  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));

  int rep = 42;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(b_vw, repeat_wf(base,rep));

  stapl::generate(c_vw, stapl::random_sequence(1000));

  zip3_vec_int_vw_tp z_vw = zip3_vec_int_vw_tp(a_vw, b_vw, c_vw); // ## 4

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(put_val_wf(zout), a_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::do_once( msg( zout, "b:" ) );
  stapl::serial_io(put_val_wf(zout), b_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::do_once( msg( zout, "c:" ) );
  stapl::serial_io(put_val_wf(zout), c_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::do_once( msg( zout, "z:" ) );
  stapl::serial_io( put_triple_wf(zout), z_vw );

  return z_vw.size();
}

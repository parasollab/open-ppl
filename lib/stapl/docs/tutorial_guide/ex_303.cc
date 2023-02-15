#include <iostream>
#include <fstream>
#include "ch3.hpp"

typedef stapl::vector< ary_int_tp > vec_ary_int_tp; // ## 1
typedef stapl::vector_view<vec_ary_int_tp> vec_ary_int_vw_tp;
typedef stapl::array< vec_int_tp > ary_vec_int_tp; // ## 2
typedef stapl::array_view<ary_vec_int_tp> ary_vec_int_vw_tp;

size_t ex_303(size_t,
              stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_303.out");

  int model = 1;
  stapl::do_once( msg( zout, "Example 303" ) );
  int result = ex_303(model, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

struct ex_303_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1) {
    stapl::iota( vw1, 0 );
  }
};

struct ex_303_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  ex_303_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1) {
    stapl::serial_io(put_val_wf(m_zout), vw1);
    stapl::do_once( msg( m_zout, "" ) );
  }
  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

size_t ex_303( size_t model,
               stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {
  size_t outer = 100 * model;
  size_t inner = 100 * model;

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner)); // ## 3

  vec_ary_int_tp a(len_vw);
  vec_ary_int_vw_tp a_vw(a);

  ary_vec_int_tp b(len_vw);
  ary_vec_int_vw_tp b_vw(b);

  stapl::map_func(ex_303_fill_wf(), a_vw ); // ## 4

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(ex_303_show_wf(zout), a_vw ); // ## 5
  stapl::do_once( msg( zout, "\n" ) );

  stapl::map_func(ex_303_fill_wf(), b_vw );

  stapl::do_once( msg( zout, "b:" ) );
  stapl::serial_io(ex_303_show_wf(zout), b_vw );
  stapl::do_once( msg( zout, "\n" ) );

  int res = stapl::map_reduce(nested_cksum_wf(), xor_un_wf(), a_vw);
  return res;
}

#include <iostream>
#include <fstream>
#include "ch3.hpp"

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector< vec_int_tp > vec_vec_int_tp; // ## 1
typedef stapl::vector_view<vec_vec_int_tp> vec_vec_int_vw_tp; // ## 2

typedef stapl::array<int> ary_int_tp;
typedef stapl::array<ary_int_tp> ary_ary_int_tp;
typedef stapl::array_view<ary_ary_int_tp> ary_ary_int_vw_tp;

size_t ex_301a(size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> &);
size_t ex_301b(size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_301.out");

  size_t model = 1;
  stapl::do_once( msg( zout, "Example 301" ) );

  zin.open("primes_100000.txt");
  int result = ex_301a(model, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  zin.close();

  zin.open("primes_1000.txt");
  result = ex_301b(model, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  zin.close();

  return EXIT_SUCCESS;
}

struct ex_301_fill_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1) {
    stapl::iota( vw1, 0 ); // ## 3
  }
};

struct ex_301_build_wf {
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1) {
    int base = 10;
    int step = 2;
    typedef stapl::sequence<int> step_wf;
    stapl::generate(vw1, step_wf(base,step));
  }
};

struct ex_301_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  ex_301_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1) {
    stapl::serial_io(put_val_wf(m_zout), vw1);
    stapl::do_once( msg(m_zout, "\n" ) );
  }
  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

struct ex_301_read_wf {
private:
  stapl::stream<ifstream> m_zin;
public:
  ex_301_read_wf (stapl::stream<ifstream> const& zin)
    : m_zin(zin)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1) {
    stapl::serial_io(get_val_wf(m_zin), vw1);
  }
  void define_type(stapl::typer& t)
  {
    t.member(m_zin);
  }
};

struct ex_301_process_wf {
  typedef void result_type;
  template <typename View1, typename View2>
  result_type operator()(View1 const &vw1, View2 const& vw2) {
    stapl::map_func(stapl::assign<vec_int_vw_tp::value_type>(),vw1,vw2); // ## 4
  }
};

struct ex_301_display_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  ex_301_display_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }
  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1) {
    stapl::serial_io(put_ndx_val_wf(m_zout),
                     stapl::counting_view<int>(vw1.size()), vw1); // ## 5
    stapl::do_once( msg(m_zout, "\n" ) );
  }
  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

size_t ex_301a( size_t model,
                stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {
  size_t outer = 100;
  size_t inner = 100;

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner)); // ## 6

  vec_vec_int_tp a(len_vw), b(len_vw); // ## 7
  vec_vec_int_vw_tp a_vw(a), b_vw(b); // ## 8

  stapl::map_func(ex_301_fill_wf(), a_vw ); // ## 9

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(ex_301_show_wf(zout), a_vw );
  stapl::do_once( msg( zout, "" ) );

  stapl::map_func(ex_301_read_wf(zin), b_vw );

  stapl::map_func(ex_301_process_wf(), a_vw, b_vw );

  stapl::do_once( msg( zout, "b:" ) );
  stapl::serial_io(ex_301_display_wf(zout), b_vw);
  stapl::do_once( msg( zout, "" ) );

  int res = stapl::map_reduce(nested_cksum_wf(), xor_un_wf(), a_vw);

  return res;
}

size_t ex_301b( size_t model,
                stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {
  size_t outer = 100;
  size_t inner = 100;

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner)); // ## 10

  ary_ary_int_tp c(len_vw), d(len_vw); // ## 11
  ary_ary_int_vw_tp c_vw(c), d_vw(d);

  stapl::map_func(ex_301_build_wf(), c_vw); // ## 12

  stapl::do_once( msg( zout, "c:" ) );
  stapl::serial_io(ex_301_show_wf(zout), c_vw);
  stapl::do_once( msg( zout, "" ) );

  stapl::map_func(ex_301_read_wf(zin), d_vw ); // ## 13

  stapl::map_func(ex_301_process_wf(), c_vw, d_vw); // ## 14

  stapl::do_once( msg( zout, "d:" ) );
  stapl::serial_io(ex_301_display_wf(zout), d_vw); // ## 15
  stapl::do_once( msg( zout, "" ) );

  int res = stapl::map_reduce(nested_cksum_wf(), xor_un_wf(), c_vw);
  return res;
}

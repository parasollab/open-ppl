#include <iostream>
#include <fstream>
#include "ch5.hpp"

#include <stapl/array.hpp>
#include <stapl/multiarray.hpp>
#include <stapl/views/reverse_view.hpp>

typedef stapl::array<int>               ary_int_tp;
typedef stapl::array_view<ary_int_tp>   ary_int_vw_tp;

typedef stapl::view_impl::reverse_view<ary_int_vw_tp> rev_ary_int_vw_tp;
typedef stapl::view_impl::reverse_view<rev_ary_int_vw_tp> rev_rev_ary_int_vw_tp;

int ex_504(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_504.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 504" ) );
  int result = ex_504(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

int ex_504(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  ary_int_tp a_ct(sz), b_ct(sz), c_ct(sz), d_ct(sz);
  ary_int_vw_tp a_ary_vw(a_ct), b_ary_vw(b_ct), c_ary_vw(c_ct), d_ary_vw(d_ct);

  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_ary_vw, step_wf(base,step));
  rev_ary_int_vw_tp a_rev_vw = rev_ary_int_vw_tp(a_ary_vw);

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(put_val_wf(zout), a_ary_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::do_once( msg( zout, "(rev a):" ) );
  stapl::serial_io(put_val_wf(zout), a_rev_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::scan(a_ary_vw, b_ary_vw, add_int_wf(), false); // ## 1

  stapl::do_once( msg( zout, "(add scan):" ) );
  stapl::serial_io(put_val_wf(zout), b_ary_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::scan(a_rev_vw, b_ary_vw, add_int_wf(), false); // ## 2

  stapl::do_once( msg( zout, "(add_scan (rev a)):" ) );
  stapl::serial_io(put_val_wf(zout), b_ary_vw );
  stapl::do_once( msg(zout, "\n" ) );

  rev_ary_int_vw_tp b_rev_vw = rev_ary_int_vw_tp(b_ary_vw); // ## 3

  stapl::do_once( msg( zout, "(rev (add_scan (rev a))):" ) );
  stapl::serial_io(put_val_wf(zout), b_rev_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::generate( c_ary_vw, stapl::random_sequence(1000) );
  rev_ary_int_vw_tp c_rev_vw = rev_ary_int_vw_tp(c_ary_vw);

  stapl::do_once( msg( zout, "c:" ) );
  stapl::serial_io(put_val_wf(zout), c_ary_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::do_once( msg( zout, "(rev c):" ) );
  stapl::serial_io(put_val_wf(zout), c_rev_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::scan(c_ary_vw, d_ary_vw, max_int_wf(), false); // ## 4

  stapl::do_once( msg( zout, "(max_scan c):" ) );
  stapl::serial_io(put_val_wf(zout), d_ary_vw );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::scan(c_rev_vw, d_ary_vw, max_int_wf(), false); // ## 5

  stapl::do_once( msg( zout, "(max_scan (rev c)):" ) );
  stapl::serial_io(put_val_wf(zout), d_ary_vw );
  stapl::do_once( msg(zout, "\n" ) );

  rev_ary_int_vw_tp d_rev_vw = rev_ary_int_vw_tp(d_ary_vw); // ## 6

  stapl::do_once( msg( zout, "(rev (max_scan (rev c))):" ) );
  stapl::serial_io(put_val_wf(zout), d_rev_vw );
  stapl::do_once( msg(zout, "\n" ) );

  int val = d_ary_vw[sz-1];
  stapl::rmi_fence(); 
  return val;
}

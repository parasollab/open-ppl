#include <iostream>
#include <fstream>
#include "ch5.hpp"

#include <stapl/multiarray.hpp>

typedef stapl::multiarray<3, int>             ary3_int_tp;    // ## 1
typedef stapl::multiarray_view<ary3_int_tp>   ary3_int_vw_tp; // ## 2

int ex_503(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin; 
  stapl::stream<ofstream> zout;
  zout.open("ex_503.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 503" ) );
  int result = ex_503(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

struct add_wf {
  typedef void result_type;
  template<typename Ref1, typename Ref2, typename Ref3>
  result_type operator()(Ref1 x, Ref2 y, Ref3 z) {
    z = x + y;
  }
};

int ex_503(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  size_t pages = 1 + rand() % (sz / 2);
  size_t rows = 1 + rand() % (sz / 2);
  size_t cols = 1 + rand() % (sz / 2);
  size_t count = pages * rows * cols;

  stapl::tuple<int,int,int> dims = stapl::make_tuple(pages,rows,cols);
  ary3_int_tp a_ct(dims), b_ct(dims), c_ct(dims);
  ary3_int_vw_tp a_vw(a_ct), b_vw(b_ct), c_vw(c_ct);

  auto a_lin_vw = stapl::linear_view(a_vw); // ## 3
  auto b_lin_vw = stapl::linear_view(b_vw);
  auto c_lin_vw = stapl::linear_view(c_vw);

  stapl::do_once( msg_val<int>( zout, "pages:", pages ) );
  stapl::do_once( msg_val<int>( zout, "rows:", rows ) );
  stapl::do_once( msg_val<int>( zout, "cols:", cols ) );

  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_lin_vw, step_wf(base,step));

  stapl::do_once( msg( zout, "a_lin:" ) );
  stapl::serial_io(show_log_ndx_wf(zout), a_lin_vw,
                   stapl::counting_view<int>(count) );
  stapl::do_once( msg(zout, "\n" ) );

  int rep = 49;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(b_lin_vw, repeat_wf(base,rep));

  stapl::do_once( msg( zout, "b_lin:" ) );
  stapl::serial_io(show_log_ndx_wf(zout), b_lin_vw,
                   stapl::counting_view<int>(count) );
  stapl::do_once( msg(zout, "\n" ) );

  stapl::map_func(add_wf(), a_lin_vw, b_lin_vw, c_lin_vw); // ## 4

  stapl::do_once( msg( zout, "c_lin:" ) );
  stapl::serial_io(show_log_ndx_wf(zout), c_lin_vw, 
                   stapl::counting_view<int>(count) );
  stapl::do_once( msg(zout, "\n" ) );

  int val = c_lin_vw[count-1];
  stapl::rmi_fence();
  return val;
}

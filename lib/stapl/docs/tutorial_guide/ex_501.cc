#include <iostream>
#include <fstream>
#include "ch5.hpp"
#include <vector>

#include <stapl/domains/indexed.hpp>
#include <stapl/vector.hpp>
#include <stapl/map.hpp>
#include <stapl/views/repeated_view.hpp>

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

typedef stapl::indexed_domain<int> ndx_dom_tp;
typedef stapl::map<int,int> map_int_tp;
typedef stapl::map_view<map_int_tp> map_int_vw_tp;

const int fib_max = 20;

int ex_501(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_501.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 501" ) );
  int result = ex_501(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

struct add_c_vec_wf {
  typedef void result_type;
  template<typename Ref1, typename Ref2, typename Ref3, typename Index>
  result_type operator()(Ref1 x, Ref2 y, Ref3 z, Index ndx) {
    z = x[ndx%fib_max] + y;
  }
};

struct add_stl_vec_wf {
  typedef void result_type;
  template<typename Ref1, typename Ref2, typename Ref3, typename Index>
  result_type operator()(Ref1 x, Ref2 y, Ref3 z, Index ndx) {
    z = x[ndx%fib_max] + y;
  }
};

struct add_stapl_map_wf {
  typedef void result_type;
  template<typename Ref1, typename Ref2, typename Ref3, typename Index>
  result_type operator()(Ref1 x, Ref2 y, Ref3 z, Index ndx) {
    z = x[ndx] + y;
  }
};

int ex_501(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  vec_int_tp a(sz), b(sz), c(sz), d(sz);
  vec_int_vw_tp a_vw(a), b_vw(b), c_vw(c), d_vw(d);

  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(put_val_wf(zout), a_vw );
  stapl::do_once( msg(zout, "\n" ) );

  int *x = fibo20;
  stapl::map_func(add_c_vec_wf(),
                  stapl::make_repeat_view(x), // ## 1
                  a_vw, b_vw, stapl::counting_view<int>(sz) );

  stapl::do_once( msg( zout, "b:" ) );
  stapl::serial_io(put_val_wf(zout), b_vw);
  stapl::do_once( msg(zout, "\n" ) );

  std::vector<int> y;
  for ( int i=0; i<fib_max; i++ ) {
    y.push_back(fibo20[i]);
  }

  stapl::map_func(add_stl_vec_wf(),
                  stapl::make_repeat_view(y), // ## 2
                  a_vw, c_vw, stapl::counting_view<int>(sz) );

  stapl::do_once( msg( zout, "c:" ) );
  stapl::serial_io(put_val_wf(zout), c_vw);
  stapl::do_once( msg(zout, "\n" ) );

  ndx_dom_tp z_dom(0,fibo20[fib_max-1]);
  map_int_tp z_ct(z_dom);
  map_int_vw_tp z_vw(z_ct);
  for ( int i=0; i<fib_max; i++ ) {
    z_vw[ fibo20[i] ] = prime_nums[i];
  }

  stapl::map_func(add_stapl_map_wf(),
                  stapl::make_repeat_view(z_vw), // ## 3
                  a_vw, d_vw, stapl::counting_view<int>(sz) );

  stapl::do_once( msg( zout, "d:" ) );
  stapl::serial_io(put_val_wf(zout), d_vw);
  stapl::do_once( msg(zout, "\n" ) );

  int val = c_vw[sz-1];
  stapl::rmi_fence();
  return val;
}

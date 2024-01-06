// view/reverse.cc

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/reverse_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::array<val_tp> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;
typedef stapl::view_impl::reverse_view<ary_int_vw_tp> rev_ary_int_vw_tp;
typedef stapl::view_impl::reverse_view<rev_ary_int_vw_tp> rev_rev_ary_int_vw_tp;

typedef stapl::plus<val_tp> add_int_wf;


stapl::exit_code stapl_main(int argc, char **argv) {

  size_t sz = 40;
  // construct container
  ary_int_tp a_ct(sz), b_ct(sz), c_ct(sz);

  // construct view over container
  ary_int_vw_tp a_ary_vw(a_ct), b_ary_vw(b_ct), c_ary_vw(c_ct);

  // construct view
  rev_ary_int_vw_tp a_rev_vw = rev_ary_int_vw_tp(a_ary_vw);

  // initialize container
  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_ary_vw, step_wf(base,step));

  // process elements through view
  stapl::scan(a_ary_vw, b_ary_vw, add_int_wf(), false);
  stapl::scan(a_rev_vw, c_ary_vw, add_int_wf(), false);

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_revvw.txt");
  stapl::serial_io( put_val_wf(zout), c_ary_vw );

  return EXIT_SUCCESS;
}

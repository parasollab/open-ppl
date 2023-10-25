// view/strided.cc

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/strided_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

struct add_wf {
  typedef void result_type;
  template<typename Ref1, typename Ref2, typename Ref3>
  result_type operator()(Ref1 x, Ref2 y, Ref3 z) {
    z = x + y;
  }
};

typedef int val_tp;
typedef stapl::array<int> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;
typedef stapl::strided_view<ary_int_vw_tp>::type str_ary_int_vw_tp;


stapl::exit_code stapl_main(int argc, char **argv) {

  size_t sz = 20;
  // construct container
  ary_int_tp a_ct(sz), b_ct(sz*2), c_ct(sz*4);

  // construct view over container
  ary_int_vw_tp a_ary_vw(a_ct), b_ary_vw(b_ct), c_ary_vw(c_ct);

  // process elements through view
  int stride = 2;
  str_ary_int_vw_tp a_str_vw = stapl::make_strided_view(a_ary_vw, stride, 0);
  str_ary_int_vw_tp b_str_vw = stapl::make_strided_view(b_ary_vw, stride, 0);
  stride = 4;
  str_ary_int_vw_tp c_str_vw = stapl::make_strided_view(c_ary_vw, stride, 0);

  // initialize container
  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_ary_vw, step_wf(base,step));

  int rep = 49;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(b_ary_vw, repeat_wf(base,rep));

  // process elements through view
  stapl::map_func(add_wf(), a_ary_vw, b_str_vw, c_str_vw);

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_strdvw.txt");
  stapl::serial_io( put_val_wf(zout), c_str_vw );

  return EXIT_SUCCESS;
}

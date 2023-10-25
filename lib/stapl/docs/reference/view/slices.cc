/* ****************************** NOT DONE ****************************** */
// view/slices.cc

#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/views/slices_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

typedef double val_tp;
typedef stapl::multiarray<4, val_tp>         ary4_dbl_tp;
typedef stapl::multiarray_view<ary4_dbl_tp>  ary4_dbl_vw_tp;

typedef stapl::plus<val_tp> add_dbl_wf;
typedef stapl::max<val_tp > max_dbl_wf;


stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container
  auto dims = stapl::make_tuple(6,6,6,6);
  ary4_dbl_tp a_ct(dims), b_ct(dims), c_ct(dims);

  // construct view over container
  ary4_dbl_vw_tp a_vw(a_ct), b_vw(b_ct), c_vw(c_ct);

  auto a_lin_vw = stapl::linear_view(a_vw);
  auto b_lin_vw = stapl::linear_view(b_vw);
  auto c_lin_vw = stapl::linear_view(c_vw);

  // initialize containers in parallel
  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_lin_vw, step_wf(base,step));

  int rep = 12;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(b_lin_vw, repeat_wf(base,rep));

  stapl::generate(c_lin_vw, stapl::random_sequence());

  // construct view over container
  auto a_sl_vw = stapl::make_slices_view<0,1,2>(a_vw);
  auto b_sl_vw = stapl::make_slices_view<0,2,3>(b_vw);
  auto c_sl_vw = stapl::make_slices_view<1,2,3>(c_vw);
 
  // process elements through view
  stapl::map_func( add_dbl_wf(), a_vw, b_vw, c_vw );

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_slcvw.txt");
  stapl::serial_io( put_val_wf(zout), c_vw );

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */

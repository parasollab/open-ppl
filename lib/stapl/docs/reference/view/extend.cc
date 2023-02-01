// view/extend.cc

#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/views/extended_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

typedef double val_tp;
typedef stapl::multiarray<3,val_tp>           ary3_dbl_tp;
typedef stapl::multiarray_view<ary3_dbl_tp>   ary3_dbl_vw_tp;

typedef stapl::plus<val_tp> add_dbl_tp;
typedef stapl::max<val_tp>  max_dbl_tp;

// CODE - finish

stapl::exit_code stapl_main(int argc, char **argv) {

  size_t sz = 40;
  // construct container
  auto a_sz = stapl::homogeneous_tuple<3>(sz);
  auto b_sz = stapl::homogeneous_tuple<3>(sz);
  get<2>(b_sz) = 1;

  ary3_dbl_tp a_ct(a_sz, 1);
  ary3_dbl_tp b_ct(b_sz, 1);

  // construct view over container
  ary3_dbl_vw_tp a_vw(a_ct);
  ary3_dbl_vw_tp b_vw(b_ct);

  // initialize container
  auto b_ext_vw = stapl::make_extended_view<2>(b_ct, sz);

  // process elements through view
  val_tp sum = stapl::map_reduce(add_dbl_tp(), max_dbl_tp(), a_vw, b_ext_vw);

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_xtndvw.txt");
  stapl::do_once( msg_val<val_tp>( zout, "Sum ", sum ) );

  return EXIT_SUCCESS;
}

// cont/multi.cc

#include <stapl/containers/multiarray/multiarray.hpp>
#include <stapl/views/multiarray_view.hpp>
#include <stapl/containers/type_traits/default_traversal.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::indexed_domain<size_t>                  vec_dom_tp;
typedef stapl::balanced_partition<vec_dom_tp>          bal_part_tp;
typedef stapl::default_traversal<3>::type              trav3_tp;
typedef stapl::nd_partition<
          stapl::tuple<bal_part_tp, bal_part_tp, bal_part_tp>,
          trav3_tp>                                    part3_tp;
typedef stapl::tuple<size_t, size_t, size_t>           gid_tp;
typedef stapl::multiarray<3, int, trav3_tp, part3_tp>  ary3_int_tp;

typedef stapl::multiarray_view<ary3_int_tp>            ary3_int_vw_tp;
typedef ary3_int_vw_tp::domain_type                    dom_tp;

typedef stapl::plus<val_tp> add_int_wf;


stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container
  stapl::tuple<size_t,size_t,size_t> dims = stapl::make_tuple(3,4,5);
  ary3_int_tp a_ct(dims), b_ct(dims), c_ct(dims);

  // construct view over container

  ary3_int_vw_tp a_vw(a_ct), b_vw(b_ct), c_vw(c_ct);

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

  stapl::iota(c_lin_vw, 0);

  // process elements in parallel
  stapl::transform( a_lin_vw, b_lin_vw, c_lin_vw, add_int_wf() );

  // print elements
  stapl::stream<ofstream> zout;
  zout.open("refman_multvw.txt");
  stapl::serial_io( put_val_wf(zout), c_vw );

  return EXIT_SUCCESS;
}

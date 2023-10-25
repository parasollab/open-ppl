// view/counting.cc

#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/views/counting_view.hpp>
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
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;


stapl::exit_code stapl_main(int argc, char **argv) {

  size_t sz = 40;
  // construct container
  vec_int_tp a_ct(sz), b_ct(sz);

  // construct view over container
  vec_int_vw_tp a_vw(a_ct), b_vw(b_ct);

  // initialize container
  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));

  // process elements through view
  stapl::map_func(add_wf(), a_vw,
                  stapl::counting_view<val_tp>(a_vw.size()),b_vw );

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_cntvw.txt");
  stapl::serial_io( put_val_wf(zout), b_vw );

  return EXIT_SUCCESS;
}

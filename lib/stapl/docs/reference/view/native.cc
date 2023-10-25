// view/native.cc

#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/views/native_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "viewhelp.hpp"
using namespace std;

struct seg_red_wf {
  typedef void result_type;
  template <typename SegRef, typename Elem>
  result_type operator()(SegRef seg, Elem elem) {
    typename SegRef::iterator iter = seg.begin();
    int tot = 0;
    while( iter != seg.end() ) {
      tot += *iter++;
    }
    elem = tot;
  }
};

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  size_t size = 20;
  // construct container
  vec_int_tp a_ct(size);
  size_t loc_nums = a_ct.get_num_locations();
  vec_int_tp b_ct(loc_nums);

  // construct view over container
  vec_int_vw_tp a_vw(a_ct), b_vw(b_ct);

  // initialize containers in parallel
  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));

  stapl::fill(b_vw, 0);

  // process elements in parallel
  stapl::map_func(seg_red_wf(), 
                  stapl::native_view(a_vw), b_vw );

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_natvw.txt");
  stapl::serial_io( put_val_wf(zout), b_vw );

  return EXIT_SUCCESS;
}

// view/overlap.cc

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/overlap_view.hpp>
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
typedef stapl::array<val_tp> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;
typedef stapl::view_impl::overlap_view_builder<ary_int_vw_tp>::view_type
  ovl_ary_int_vw_tp;

stapl::exit_code stapl_main(int argc, char **argv) {

  size_t size = 40;

  int left = 1;
  int right = 1;
  int core = 1;

  // construct container
  ary_int_tp a_ct(size), b_ct(size - (left+right));

  // construct view over container
  ary_int_vw_tp a_vw(a_ct), b_vw(b_ct);

  // construct overlap view
  ovl_ary_int_vw_tp a_ovl_vw = make_overlap_view(a_vw, core,left,right);

  // initialize containers in parallel
  stapl::iota(a_vw, 0);
  stapl::fill(b_vw, 0);

  // process elements through view
  stapl::map_func(seg_red_wf(), a_ovl_vw, b_vw );

  // print elements
  stapl::stream<ofstream> zout;
  zout.open("refman_aryvw.txt");
  stapl::serial_io( put_val_wf(zout), a_vw );

  return EXIT_SUCCESS;
}

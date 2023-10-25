/* ****************************** NOT DONE ****************************** */
// view/segment.cc

#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/views/segmented_view.hpp>
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
typedef stapl::array<val_tp>               ary_int_tp;
typedef stapl::array_view<ary_int_tp>   ary_int_vw_tp;
 
typedef stapl::splitter_partition<ary_int_vw_tp::domain_type>
        seg_ary_splitter;
typedef stapl::segmented_view<ary_int_vw_tp, seg_ary_splitter>
        seg_ary_vw_tp;
typedef stapl::splitter_partition<ary_int_vw_tp::domain_type> seg_ary_splitter;
typedef stapl::segmented_view<ary_int_vw_tp, seg_ary_splitter> seg_ary_vw_tp;

size_t seg_cnt = 15;
size_t seg_len[15] = { 2, 3, 2, 3, 5, 2, 3, 5, 7, 1, 2, 3, 4, 3, 5};


stapl::exit_code stapl_main(int argc, char **argv) {

  size_t sz = 50;
  // construct container
  ary_int_tp a(sz);

  // construct view over container
  ary_int_vw_tp a_vw(a);

  // initialize container
  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));

  // construct view over container
  std::vector<size_t> stdq(seg_cnt-1);

  stapl::do_once([&](void) {
    for ( size_t i= 0; i<seg_cnt-1; i++ ) {
      stdq[i] = seg_len[i];
    }
    std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());
  });

  seg_ary_vw_tp seg_a_vw( a_vw,
                 seg_ary_splitter(a_vw.domain(), stdq,true));

  ary_int_tp b(seg_cnt), c(seg_cnt);
  ary_int_vw_tp b_vw(b), c_vw(c);

  // process elements through view
  stapl::map_func(seg_red_wf(), seg_a_vw, b_vw );

  // print container elements
  stapl::stream<ofstream> zout;
  zout.open("refman_segvw.txt");
  stapl::serial_io( put_val_wf(zout), b_vw );

  return EXIT_SUCCESS;
}
/* ****************************** NOT DONE ****************************** */

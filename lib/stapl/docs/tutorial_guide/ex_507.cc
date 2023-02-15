#include <iostream>
#include <fstream>
#include "ch5.hpp"

#include <stapl/array.hpp>
#include <stapl/views/segmented_view.hpp>

typedef stapl::array<int>               ary_int_tp;
typedef stapl::array_view<ary_int_tp>   ary_int_vw_tp;

typedef stapl::splitter_partition<ary_int_vw_tp::domain_type>
        seg_ary_splitter;
typedef stapl::segmented_view<ary_int_vw_tp, seg_ary_splitter>
        seg_ary_vw_tp;
typedef stapl::splitter_partition<ary_int_vw_tp::domain_type> seg_ary_splitter;
typedef stapl::segmented_view<ary_int_vw_tp, seg_ary_splitter> seg_ary_vw_tp;

int ex_507(size_t,
           stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  zin.open("factors_10000.txt");
  stapl::stream<ofstream> zout;
  zout.open("ex_507.out");

  size_t size = 100;
  stapl::do_once( msg( zout, "Example 507" ) );
  int result = ex_507(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

struct ex_507_seg_flags_wf { // ## 1
  typedef int result_type;
  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 key, Ref2 seg_flag) {
    int prev = key[0];
    int count = 0;
    size_t limit = key.size();
    for( size_t i=1; i < limit; i++ ) {
      if( prev != key[i] ) {
        prev = key[i];
        seg_flag[i-1] = 1;
        count++;
      }
    }
    seg_flag[limit-1] = 1;
    count++;
    return count;
  }
};

struct ex_507_seg_counts_wf { // ## 2
  typedef void result_type;
  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 seg_flag, Ref2 seg_cnt) {
    int count = 1;
    size_t j = 0;
    size_t limit = seg_flag.size();
    for( size_t i=0; i<limit; i++ ) {
      if( seg_flag[i] == 1 ) {
        seg_cnt[j++] = count;
        count = 1;
      } else {
        count++;
      }
    }
  }
};

struct ex_507_seg_red_wf {
  typedef void result_type;
  template <typename SegRef, typename Elem>
  result_type operator()(SegRef seg, Elem elem) {
    typename SegRef::iterator iter = seg.begin();
    int tot = 0;
    while( iter != seg.end() ) {
      tot += *iter++; // ## 3
    }
    elem = tot; // ## 4
  }
};

int ex_507(size_t sz,
           stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  ary_int_tp a(sz);
  ary_int_vw_tp a_vw(a);

  int base = 0;
  int step = 10;
  typedef stapl::sequence<int> step_wf;
  stapl::generate(a_vw, step_wf(base,step));

  stapl::do_once( msg( zout, "a:" ) );

  stapl::serial_io(put_val_wf(zout), a_vw);
  stapl::do_once( msg( zout, "\n" ) );

  ary_int_tp key(sz), fac(sz), powers(sz);
  ary_int_vw_tp key_vw(key), fac_vw(fac), pow_vw(powers);
  stapl::serial_io(get_triple_wf(zin), key_vw, fac_vw, pow_vw ); // ## 6

  ary_int_tp seg_flag(sz);
  ary_int_vw_tp seg_flag_vw(seg_flag);
  int count = stapl::do_once( ex_507_seg_flags_wf(), key_vw, seg_flag_vw ); // ## 7

  stapl::do_once( msg_val<int>( zout, "key count ", key_vw.size() ) );

  stapl::do_once( msg( zout, "seg_flag_vw:" ) );
  stapl::serial_io(put_val_wf(zout), seg_flag_vw);
  stapl::do_once( msg(zout, "\n" ) );

  ary_int_tp seg_cnt(count);
  ary_int_vw_tp seg_cnt_vw(seg_cnt);
  stapl::do_once( ex_507_seg_counts_wf(), seg_flag_vw, seg_cnt_vw ); // ## 8

  std::vector<size_t> stdq(seg_cnt.size()-1); // ## 9
  std::copy(seg_cnt.begin(), seg_cnt.end()-1, stdq.begin()); // ## 10
  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());

  seg_ary_vw_tp seg_a_vw( a_vw,
                 seg_ary_splitter(a_vw.domain(), stdq, false)); // ## 11

  stapl::do_once( msg_val<int>( zout, "segment count ", seg_a_vw.size() ) );

  ary_int_tp b(count);
  ary_int_vw_tp b_vw(b);

  stapl::map_func(ex_507_seg_red_wf(), seg_a_vw, b_vw ); // ## 12

  stapl::do_once( msg( zout, "b:" ) );
  stapl::serial_io(put_val_wf(zout), b_vw);
  stapl::do_once( msg(zout, "\n" ) );

  return 0;
}













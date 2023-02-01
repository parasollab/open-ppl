#include <iostream>
#include <fstream>
#include "ch3.hpp"

#include <stapl/views/segmented_view.hpp>

typedef stapl::vector<map_int_tp> vec_map_int_tp; // ## 1
typedef stapl::vector_view<vec_map_int_tp> vec_map_int_vw_tp;
typedef stapl::array<map_int_tp> ary_map_int_tp; // ## 2
typedef stapl::array_view<ary_map_int_tp> ary_map_int_vw_tp;

typedef stapl::splitter_partition<vec_int_vw_tp::domain_type>
        seg_vec_splitter; // ## 3
typedef stapl::segmented_view<vec_int_vw_tp, seg_vec_splitter>
        seg_vec_vw_tp; // ## 4
typedef stapl::splitter_partition<ary_int_vw_tp::domain_type> seg_ary_splitter;
typedef stapl::segmented_view<ary_int_vw_tp, seg_ary_splitter> seg_ary_vw_tp;

extern int countbits(int);

size_t ex_304a(size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t ex_304b(size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  zin.open("factors_10000.txt");
  stapl::stream<ofstream> zout;
  zout.open("ex_304.out");

  size_t model = 1;
  stapl::do_once( msg( zout, "Example 304" ) );
  int result = ex_304a(model, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  result = ex_304b(model, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

struct ex_304a_fill_wf {
private:
  size_t outer;
  size_t inner;
public:
  ex_304a_fill_wf(size_t o, size_t i)
    : outer(o), inner(i)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a) {
    for ( size_t i = 0; i < outer; i++ ) {
      for ( size_t j = 0; j < inner; j++ ) {
        int key = prime_nums[j];
        a[i][key] = countbits(key); // ## 5
      }
    }
  }
  void define_type(stapl::typer& t) {
    t.member(outer);
    t.member(inner);
  }
};

struct ex_304a_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem) {
    int val = stapl::map_reduce( inner_map_wf(), add_int_wf(), elem);
    return val;
  }
};

size_t ex_304a( size_t model,
                stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {
  size_t outer = 100 * model;
  size_t inner = 100 * model;

  vec_map_int_tp a(outer), b(outer); // ## 6
  vec_map_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once( ex_304a_fill_wf(outer,inner), a_vw );

  stapl::map_func(ex_304a_process_wf(), a_vw );

  stapl::do_once( msg( zout, "a:" ) );
#ifdef GFORGE_1401_FIXED
  stapl::serial_io(show_indexable_map_wf(zout), a_vw );
#endif
  stapl::do_once( msg( zout, "" ) );

  size_t cksum = stapl::map_reduce(inner_map_cksum_wf(), xor_un_wf(), a_vw);
  return cksum;
}

struct ex_304b_seg_flags_wf {
  typedef int result_type;
  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 key, Ref2 seg_flag) {
    int prev = key[0];
    int count = 0;
    int limit = key.size();
    for( int i=1; i < limit; i++ ) {
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

struct ex_304b_seg_counts_wf {
  typedef void result_type;
  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 seg_flag, Ref2 seg_cnt) {
    int count = 1;
    size_t j = 0;
    int limit = seg_flag.size();
    for( int i=0; i<limit; i++ ) {
      if( seg_flag[i] == 1 ) {
        seg_cnt[j++] = count;
        count = 1;
      } else {
        count++;
      }
    }
  }
};

struct ex_304b_build_wf {
  typedef void result_type;
  template <typename SegRef1, typename SegRef2, typename View1>
  result_type operator()(SegRef1 seg1, SegRef2 seg2, View1 vw) {
    typename SegRef1::iterator iter1 = seg1.begin(); // ## 7
    typename SegRef2::iterator iter2 = seg2.begin();
    while( iter1 != seg1.end() ) {
      vw[*iter1++] = *iter2++; // ## 8
    }
  }
};

struct ex_304b_process_wf {
  typedef void result_type;
  template <typename Elem1, typename Elem2>
  result_type operator()(Elem1 elem1, Elem2 elem2) {
    elem2 = elem1.size(); // ## 9
  }
};

size_t ex_304b( size_t model,
                stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t outer = 10000 * model;
  ary_int_tp key(outer), fac(outer), powers(outer);
  ary_int_vw_tp key_vw(key), fac_vw(fac), pow_vw(powers);

  stapl::serial_io(get_triple_wf(zin), key_vw, fac_vw, pow_vw ); // ## 10

  ary_int_tp seg_flag(outer);
  ary_int_vw_tp seg_flag_vw(seg_flag);
  int count = stapl::do_once( ex_304b_seg_flags_wf(), key_vw, seg_flag_vw ); // ## 11

  ary_int_tp seg_cnt(count);
  ary_int_vw_tp seg_cnt_vw(seg_cnt);
  stapl::do_once( ex_304b_seg_counts_wf(), seg_flag_vw, seg_cnt_vw ); // ## 12

  std::vector<size_t> stdq(seg_cnt.size()-1); // ## 13

  std::copy(seg_cnt.begin(), seg_cnt.end()-1, stdq.begin()); // ## 14

  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());

  seg_ary_vw_tp seg_fac_vw( fac_vw,
                 seg_ary_splitter(fac_vw.domain(), stdq, false)); // ## 15
  seg_ary_vw_tp seg_pow_vw( pow_vw,
                 seg_ary_splitter(pow_vw.domain(), stdq, false));

  ary_map_int_tp mv(seg_fac_vw.size()); // ## 16
  ary_map_int_vw_tp mv_vw(mv);

  ary_int_tp mf(seg_fac_vw.size());
  ary_int_vw_tp mf_vw(mf);

  stapl::map_func(ex_304b_build_wf(), seg_fac_vw, seg_pow_vw, mv_vw ); // ## 17

  stapl::map_func(ex_304b_process_wf(), mv_vw, mf_vw );

  stapl::do_once( msg( zout, "mf:" ) );
  stapl::serial_io(put_val_wf(zout), mf_vw );
  stapl::do_once( msg( zout, "" ) );

  stapl::do_once( msg( zout, "mv:" ) );
#ifdef GFORGE_1401_FIXED
  stapl::serial_io(show_indexable_map_wf(zout), mv_vw );
#endif
  stapl::do_once( msg( zout, "" ) );

  int res = stapl::map_reduce(inner_map_cksum_wf(), xor_un_wf(), mv_vw); // ## 18
  return res;
}

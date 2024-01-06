#include <iostream>
#include <fstream>
#include "ch4.hpp"

#define ARB_DIST_ACTIVE 1

#include <stapl/views/segmented_view.hpp>

typedef stapl::vector<int> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;
typedef stapl::array<int> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;

typedef stapl::map<int,int> map_int_tp;
typedef stapl::map_view<map_int_tp> map_int_vw_tp;
typedef stapl::map< int, vec_int_tp > map_vec_int_tp; // ## 1
typedef stapl::map_view<map_vec_int_tp> map_vec_int_vw_tp;
typedef stapl::map< int, ary_int_tp > map_ary_int_tp; // ## 2
typedef stapl::map_view<map_ary_int_tp> map_ary_int_vw_tp;

typedef stapl::indexed_domain<int> ndx_dom_tp;
typedef stapl::array<size_t> ary_sz_tp;
typedef stapl::array_view<ary_sz_tp> ary_sz_vw_tp;
typedef stapl::splitter_partition<vec_int_vw_tp::domain_type> seg_vec_splitter;
typedef stapl::segmented_view<vec_int_vw_tp, seg_vec_splitter> seg_vec_vw_tp;
typedef stapl::splitter_partition<ary_int_vw_tp::domain_type> seg_ary_splitter;
typedef stapl::segmented_view<ary_int_vw_tp, seg_ary_splitter> seg_ary_vw_tp;

extern int countbits(int);

size_t ex_405a(size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t ex_405b(size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_405.out");

  size_t model = 1;
  stapl::do_once( msg( zout, "Example 405" ) );

  zin.open("factors_1000.txt");
  int result = ex_405a(model, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  zin.close();

  zin.open("factors_1000.txt");
  result = ex_405b(model, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  zin.close();
  return EXIT_SUCCESS;
}

struct ex_405a_fill_wf {
private:
  size_t outer;
  size_t inner;
public:
  ex_405a_fill_wf(size_t o, size_t i)
    : outer(o), inner(i)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a) {
    if ( outer <= data_cnt ) {
      for ( size_t i = 0; i < outer; i++ ) {
        int key = prime_nums[i];
        size_t max_size = a[key].size();
        for ( size_t j = 0; j < inner; j++ ) {
          int key = prime_nums[i];
          size_t index = rand_nums[j];
          int value = countbits(index);
          if (index+1 >= max_size)
          {
            max_size = index+1;
            a[key].resize(max_size);
          }
          a[key][index] = value; // ## 3
        }
      }
    } else {
      for ( size_t i = 0; i < outer; i += data_cnt ) {
        for ( size_t j = 0; j < inner; j++ ) {
          for ( size_t k = 0; k < data_cnt; k++ ) {
            int key = prime_nums[k] * prime_nums[data_cnt-k];
            int index = rand_nums[j];
            int value = countbits(index);
            a[key][index] = value;
          }
        }
      }
    }
  }
  void define_type(stapl::typer& t) {
    t.member(outer);
    t.member(inner);
  }
};

struct ex_405a_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem) {
    typename Element::second_reference map_arg = elem.second; // ## 4
    int val = stapl::map_reduce( id_int_wf(), add_int_wf(), map_arg); // ## 5
    return val;
  }
};

size_t ex_405a( size_t model,
                stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {
  size_t outer_a = 100 * model;
  size_t inner = 100 * model;

  ary_sz_tp len(outer_a);
  ary_sz_vw_tp len_vw(len);

  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner));

  ndx_dom_tp map_dom(0, 16277216);
  map_vec_int_tp a(map_dom), b(map_dom); // ## 6
  map_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(ex_405a_fill_wf(outer_a, inner), a_vw);

#ifndef BUG
stapl::rmi_fence();
return 0;
#endif

  stapl::map_func(ex_405a_process_wf(), a_vw );

  stapl::do_once( msg(zout, "a:" ) );
  stapl::serial_io(show_map_indexable_wf(zout), a_vw );
  stapl::do_once( msg(zout, "" ) );

  size_t cksum = stapl::map_reduce(outer_map_cksum_wf(), xor_un_wf(), a_vw);

  return cksum;
}

struct ex_405b_seg_flags_wf { // ok
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
#if 0
    seg_flag[limit-1] = 1;
    count++;
#endif
    return count;
  }
};

struct ex_405b_seg_counts_wf {
  typedef void result_type;
  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 seg_flag, Ref2 seg_cnt) {
    int count = 1;
    size_t j = 0;
    size_t limit = seg_flag.size();
    for( size_t i=0; i<limit; i++ ) {
      if( seg_flag[i] == 1 ) {
        seg_cnt[j] = count;
        j++;
        count = 1;
      } else {
        count++;
      }
    }
  }
};

struct ex_405b_build_wf {
  typedef void result_type;
  template <typename Ref1, typename Ref2, typename View1>
  result_type operator()(Ref1 ref1, Ref2 ref2, View1 vw) {
    vw[ref1].resize(ref2);
  }
};

struct ex_405b_fill_wf {
  typedef void result_type;
  template <typename SegRef1, typename SegRef2, typename View1>
  result_type operator()(SegRef1 seg1, SegRef2 seg2, View1 vw) {
    auto iter1 = seg1.begin(); // ## 7
    auto iter2 = seg2.begin();
    while( iter1 != seg1.end() ) {
      int x = *iter1++;
      int y = *iter2++;
      vw[x].resize(y); // ## 8
    }
  }
};

size_t ex_405b( size_t model,
                stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t outer_b = 10000 * model;
  ary_int_tp key(outer_b), fac(outer_b), bits(outer_b);
  ary_int_vw_tp key_vw(key), fac_vw(fac), bits_vw(bits);
  stapl::serial_io(get_triple_wf(zin), key_vw, fac_vw, bits_vw );

  ary_int_tp seg_flag(outer_b);
  ary_int_vw_tp seg_flag_vw(seg_flag);
  int count = stapl::do_once( ex_405b_seg_flags_wf(), key_vw, seg_flag_vw ); // ## 10

  ary_int_tp seg_cnt(count);
  ary_int_vw_tp seg_cnt_vw(seg_cnt);
  stapl::do_once( ex_405b_seg_counts_wf(), seg_flag_vw, seg_cnt_vw ); // ## 11

  std::vector<size_t> stdq(seg_cnt.size()-1); // ## 12
  std::copy(seg_cnt.begin(), seg_cnt.end()-1, stdq.begin());
  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());

  seg_ary_vw_tp seg_fac_vw( fac_vw,
                 seg_ary_splitter(fac_vw.domain(), stdq, false));
  seg_ary_vw_tp seg_key_vw( key_vw,
                 seg_ary_splitter(key_vw.domain(), stdq, false));

  ary_int_tp uniq_first(key_vw.size());
  ary_int_vw_tp uniq_first_vw(uniq_first);
  stapl::unique_copy( key_vw, uniq_first_vw );

  ary_int_tp uniq_key(count);
  ary_int_vw_tp uniq_key_vw(uniq_key);
  stapl::copy_n( uniq_first_vw, uniq_key_vw, count );

  ndx_dom_tp map_dom(0, 16277216);
  map_ary_int_tp mv(map_dom);
  map_ary_int_vw_tp mv_vw(mv);
  map_int_tp mf(map_dom);
  map_int_vw_tp mf_vw(mf);

  stapl::map_func(ex_405b_build_wf(), uniq_key_vw, seg_cnt_vw,
                  stapl::make_repeat_view(mv_vw) ); // ## 13

  stapl::map_func(ex_405b_fill_wf(), seg_key_vw, seg_fac_vw,
                  stapl::make_repeat_view(mv_vw) ); // ## 14

  stapl::do_once( msg(zout, "mv:" ) );
  stapl::serial_io(show_map_indexable_wf(zout), mv_vw );
  stapl::do_once( msg(zout, "" ) );

  int res = stapl::map_reduce(outer_map_cksum_wf(), xor_un_wf(), mv_vw);
  return res;
}

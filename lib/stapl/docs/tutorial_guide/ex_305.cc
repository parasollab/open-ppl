#include <iostream>
#include <fstream>
#include "ch3.hpp"

#include <stapl/views/repeated_view.hpp>
#include <stapl/views/segmented_view.hpp>

typedef stapl::map< int, vec_int_tp > map_vec_int_tp; // ## 1
typedef stapl::map_view<map_vec_int_tp> map_vec_int_vw_tp;
typedef stapl::map< int, ary_int_tp > map_ary_int_tp; // ## 2
typedef stapl::map_view<map_ary_int_tp> map_ary_int_vw_tp;

typedef stapl::splitter_partition<vec_int_vw_tp::domain_type> seg_vec_splitter;
typedef stapl::segmented_view<vec_int_vw_tp, seg_vec_splitter> seg_vec_vw_tp;
typedef stapl::splitter_partition<ary_int_vw_tp::domain_type> seg_ary_splitter;
typedef stapl::segmented_view<ary_int_vw_tp, seg_ary_splitter> seg_ary_vw_tp;

extern int countbits(int);

size_t ex_305a(size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );
size_t ex_305b(size_t,
               stapl::stream<ifstream> &, stapl::stream<ofstream> & );

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  stapl::stream<ofstream> zout;
  zout.open("ex_305.out");

  size_t model = 1;
  stapl::do_once( msg( zout, "Example 305" ) );

  zin.open("factors_10000.txt");
  int result = ex_305a(model, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  zin.close();

  zin.open("factors_10000.txt");
  result = ex_305b(model, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  zin.close();
  return EXIT_SUCCESS;
}

struct ex_305a_fill_wf {
private:
  size_t outer;
  size_t inner;
public:
  ex_305a_fill_wf(size_t o, size_t i)
    : outer(o), inner(i)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a) {
    if ( outer <= data_cnt ) {
      for ( size_t i = 0; i < outer; i++ ) {
        int key = prime_nums[i];
        size_t max_size = a[key].size(); // ## 3
        for ( size_t j = 0; j < inner; j++ ) {
          size_t index = rand_nums[j];
          int value = countbits(index);
          if (index+1 >= max_size)
          {
            max_size = index+1;
            a[key].resize(max_size);
          }
          a[key][index] = value; // ## 4
        }
      }
    } else {
      for ( size_t i = 0; i < outer; i += data_cnt ) {
        for ( size_t j = 0; j < inner; j++ ) {
          for ( size_t k = 0; k < data_cnt; k++ ) {
            int key = prime_nums[k] * prime_nums[data_cnt-k];
            size_t index = rand_nums[j];
            int value = countbits(index);
            if (index+1 >= a[key].size())
              a[key].resize(index+1);
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

struct ex_305a_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem) {
    typename Element::second_reference map_arg = elem.second; // ## 5
    int val = stapl::reduce(map_arg, add_int_wf()); // ## 6
    return val;
  }
};

size_t ex_305a( size_t model,
                stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {
  size_t outer_a = 100 * model;
  size_t inner = 100 * model;

  ndx_dom_tp map_dom(0, 16277216);
  map_vec_int_tp a(map_dom), b(map_dom); // ## 7
  map_vec_int_vw_tp a_vw(a), b_vw(b);

  stapl::do_once(ex_305a_fill_wf(outer_a, inner), a_vw);

  stapl::map_func(ex_305a_process_wf(), a_vw );

  stapl::do_once( msg(zout, "a:" ) );
  stapl::serial_io(show_map_indexable_wf(zout), a_vw );
  stapl::do_once( msg(zout, "" ) );

  size_t cksum = stapl::map_reduce(outer_map_cksum_wf(), xor_un_wf(), a_vw);

  return cksum;
}

struct ex_305b_build_wf {
  typedef void result_type;
  template <typename SegKeyRef, typename SegFacRef, typename MapRef>
  result_type
  operator()(SegKeyRef seg_key, SegFacRef seg_fac, MapRef map_ary_vw) {
    typename SegKeyRef::value_type key = *seg_key.begin(); // ## 8

    map_ary_vw[key].resize(seg_fac.size());

    size_t j = 0;
    typename SegFacRef::iterator iter_fac = seg_fac.begin(); // ## 9
    while (iter_fac != seg_fac.end()) {
      map_ary_vw[key][j++] = *iter_fac++; // ## 10
    }
  }
};

struct ex_305b_seg_flags_wf {
  typedef int result_type;
  template <typename Ref1, typename Ref2>
  result_type operator()(Ref1 key, Ref2 seg_flag) {
    int prev = key[0];
    int count = 0;
    int limit = key.size();
    for ( int i=1; i < limit; i++ ) {
      if ( prev != key[i] ) {
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

struct ex_305b_seg_counts_wf {
  typedef int result_type;
  template <typename Ref1, typename Ref2>
  void operator()(Ref1 seg_flag, Ref2 seg_cnt) {
    int count = 1;
    size_t j = 0;
    int limit = seg_flag.size();
    for ( int i=0; i<limit; i++ ) {
      if ( seg_flag[i] == 1 ) {
        seg_cnt[j++] = count;
        count = 1;
      } else {
        count++;
      }
    }
  }
};

struct ex_305b_process_wf {
  typedef void result_type;
  template <typename ElemRef, typename MapRef>
  result_type operator()(ElemRef elem, MapRef map_int) {
    typename ElemRef::first_reference key = elem.first;
    typename ElemRef::second_reference vec = elem.second;
    map_int[key] = vec.size(); // ## 11
  }
};

size_t ex_305b( size_t model,
                stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout ) {

  size_t outer_b = 10000 * model;
  ary_int_tp key(outer_b), fac(outer_b), powers(outer_b);
  ary_int_vw_tp key_vw(key), fac_vw(fac), pow_vw(powers);

  stapl::serial_io(get_triple_wf(zin), key_vw, fac_vw, pow_vw );

  ary_int_tp seg_flag(outer_b);
  ary_int_vw_tp seg_flag_vw(seg_flag);
  int count =
    stapl::do_once( ex_305b_seg_flags_wf(), key_vw, seg_flag_vw ); // ## 12

  ary_int_tp seg_cnt(count);
  ary_int_vw_tp seg_cnt_vw(seg_cnt);
  stapl::do_once( ex_305b_seg_counts_wf(), seg_flag_vw, seg_cnt_vw ); // ## 13

  std::vector<size_t> stdq(seg_cnt.size()-1); // ## 14
  std::copy(seg_cnt.begin(), seg_cnt.end()-1, stdq.begin());

  std::partial_sum(stdq.begin(), stdq.end(), stdq.begin());
  seg_ary_vw_tp seg_fac_vw( fac_vw,
                 seg_ary_splitter(fac_vw.domain(), stdq, false));
  seg_ary_vw_tp seg_key_vw( key_vw,
                 seg_ary_splitter(key_vw.domain(), stdq, false));

  ndx_dom_tp map_dom(0, 16277216);
  map_ary_int_tp mv(map_dom);
  map_ary_int_vw_tp mv_vw(mv);
  map_int_tp mf(map_dom);
  map_int_vw_tp mf_vw(mf);

  stapl::map_func(ex_305b_build_wf(), seg_key_vw, seg_fac_vw,
                  stapl::make_repeat_view(mv_vw) ); // ## 15

  stapl::map_func(ex_305b_process_wf(), mv_vw,
                  stapl::make_repeat_view(mf_vw) );

  stapl::do_once( msg(zout, "mv:" ) );
  stapl::serial_io(show_map_indexable_wf(zout), mv_vw );
  stapl::do_once( msg(zout, "" ) );

  stapl::do_once( msg(zout, "mf:" ) );
  stapl::map_func(put_map_val_wf(zout), mf_vw );
  stapl::do_once( msg(zout, "" ) );

  int res = stapl::map_reduce(outer_map_cksum_wf(), xor_un_wf(), mv_vw);
  return res;
}

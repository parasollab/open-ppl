#include <iostream>
#include <fstream>
#include "ch3.hpp"

typedef stapl::map<int, stapl::map<int,int> > map_map_int_tp; // ## 1
typedef stapl::map_view<map_map_int_tp> map_map_int_vw_tp;

size_t ex_302(size_t,
              stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  zin.open("factors_10000.txt");
  stapl::stream<ofstream> zout;
  zout.open("ex_302.out");

  size_t model = 1;
  stapl::do_once( msg( zout, "Example 302" ) );
  int result = ex_302(model, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

struct ex_302_fill_wf {
private:
  size_t outer;
  size_t inner;
public:
  ex_302_fill_wf(size_t o, size_t i)
    : outer(o), inner(i)
  { }

  typedef void result_type;
  template <typename Data>
  result_type operator()(Data &a) {
    if (outer <= data_cnt ) {
      for (size_t i = 0; i < outer; i++ ) {
        for (size_t j = 0; j < inner; j++ ) {
          int outer_key = prime_nums[i];
          int inner_key = rand_nums[j];
          int value = rand_nums[i] % (j+1);
          a[outer_key][inner_key] = value; // ## 2
        }
      }
    } else {
      for (size_t i = 0; i < outer; i += data_cnt ) {
        for (size_t j = 0; j < inner; j++ ) {
          for (size_t k = 0; k < data_cnt; k++ ) {
            int outer_key = prime_nums[k]*prime_nums[data_cnt-k];
            int inner_key = rand_nums[j];
            int value = fibo20[i%20];
            a[outer_key][inner_key] = value;
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

struct ex_302_process_wf {
  typedef int result_type;
  template <typename Element>
  result_type operator()(Element elem) {
    return elem.second.size(); // ## 3
  }
};

struct ex_302_insert_wf {
private:
  size_t limit;
public:
  ex_302_insert_wf(size_t sz)
    : limit(sz)
  { }

  typedef void result_type;
  template <typename Ref1, typename Ref2, typename Ref3, typename Data>
  result_type operator()( Ref1 left, Ref2 right, Ref3 value, Data &b ) {
    for (size_t i = 0; i < limit; i++ ) {
      b[left[i]][right[i]] = value[i]; // ## 4
    }
  }
  void define_type(stapl::typer& t) {
    t.member(limit);
  }
};

size_t ex_302( size_t model,
               stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {
  size_t outer_a = 100 * model;
  size_t inner_a = 100 * model;

  ndx_dom_tp map_dom_a(0, prime_nums[data_cnt-1]); // ## 5
  map_map_int_tp a(map_dom_a); // ## 6
  map_map_int_vw_tp a_vw(a); // ## 7

  stapl::do_once(ex_302_fill_wf(outer_a, inner_a), a_vw ); // ## 8

  stapl::map_func(ex_302_process_wf(), a_vw );

  stapl::do_once( msg( zout, "a:" ) );
  stapl::serial_io(show_outer_map_wf(zout), a_vw );
  stapl::do_once( msg( zout, "") );

  size_t size_b = data_cnt;
  ary_sz_tp lkey(size_b), rkey(size_b), val(size_b); // ## 9
  ary_sz_vw_tp lkey_vw(lkey), rkey_vw(rkey), val_vw(val);

  stapl::do_once( msg( zout, "lkey, rkey, val" ) );
  stapl::serial_io(get_triple_wf(zin), lkey_vw, rkey_vw, val_vw ); // ## 10
  stapl::do_once( msg( zout, "") );

  ndx_dom_tp map_dom_b(0, 16777216); // ## 11
  map_map_int_tp b(map_dom_b);
  map_map_int_vw_tp b_vw(b);

  stapl::do_once(ex_302_insert_wf(size_b), lkey_vw, rkey_vw, val_vw, b_vw );

  stapl::do_once( msg( zout, "b:" ) );
  stapl::serial_io(show_outer_map_wf(zout), b_vw );
  stapl::do_once( msg( zout, "") );

  int res = stapl::map_reduce(multi_map_cksum_wf(), xor_un_wf(), a_vw);
  return res;
}

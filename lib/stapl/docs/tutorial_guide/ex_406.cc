#include <iostream>
#include <fstream>
#include "ch4.hpp"
#include <stapl/views/set_view.hpp>
#include <stapl/containers/set/set.hpp>

typedef stapl::set<int>                         set_int_tp;
typedef stapl::set_view<set_int_tp>             set_int_vw_tp;

typedef stapl::array<set_int_tp>                ary_set_int_tp; // ## 1
typedef stapl::array_view<ary_set_int_tp>       ary_set_int_vw_tp;
typedef stapl::vector<set_int_tp>               vec_set_int_tp; // ## 2
typedef stapl::vector_view<vec_set_int_tp>      vec_set_int_vw_tp;

size_t ex_406(size_t,
              stapl::stream<ifstream> &, stapl::stream<ofstream> &);

stapl::exit_code stapl_main(int argc, char **argv) {
  stapl::stream<ifstream> zin;
  zin.open("primes_10000.txt");
  stapl::stream<ofstream> zout;
  zout.open("ex_406.out");

  size_t size = 1;
  stapl::do_once( msg( zout, "Example 406" ) );
  int result = ex_406(size, zin, zout);
  stapl::do_once( msg_val<int>( zout, "Result ", result ) );
  return EXIT_SUCCESS;
}

struct ex_406_gen_wf {
private:
  size_t m_card;
public:
  ex_406_gen_wf(size_t c)
    : m_card(c)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1) {
    for ( size_t i = 0; i < m_card; i++ ) {
      int key = prime_nums[rand_nums[i]];
      vw1.insert( key ); // ## 3
    }
  }
  void define_type(stapl::typer& t) {
    t.member(m_card);
  }
};

struct ex_406_fill_wf {
  typedef void result_type;
  template <typename View1, typename Elem>
  result_type operator()(View1 const &vw1, Elem count) {
    stapl::do_once(ex_406_gen_wf(count), vw1 ); // ## 4
  }
};

struct ex_406_process_wf {
  typedef int result_type;
  template <typename View1>
  result_type operator()(View1 const &vw1) {
    return vw1.size(); // ## 5
  }
};

struct ex_406_show_wf {
private:
  stapl::stream<ofstream> m_zout;
public:
  ex_406_show_wf(stapl::stream<ofstream> const& zout)
    : m_zout(zout)
  { }

  typedef void result_type;
  template <typename View1>
  result_type operator()(View1 vw1) {
#ifdef GFORGE_1309_FIXED
    set_int_vw_tp vw2 = vw1;
    stapl::serial_io(put_val_wf(m_zout), vw1);
    stapl::do_once( msg( zout, "\n") );
#endif
  }
  void define_type(stapl::typer& t) {
    t.member(m_zout);
  }
};

size_t ex_406( size_t model,
              stapl::stream<ifstream> &zin, stapl::stream<ofstream> &zout) {

  size_t outer = 100 * model; // ## 6
  size_t inner = 100 * model; // ## 7

  ary_sz_tp len(outer);
  ary_sz_vw_tp len_vw(len);
  stapl::map_func(roll_wf(), len_vw, stapl::make_repeat_view(inner)); // ## 8

  ary_set_int_tp a(outer); // ## 9
  ary_set_int_vw_tp a_vw(a);

  stapl::map_func( ex_406_fill_wf(), a_vw, len_vw ); // ## 10

  int res = stapl::map_reduce(ex_406_process_wf(), max_int_wf(), // ## 11
                              a_vw );

  stapl::do_once( msg( zout, "a:") );

  stapl::map_func(ex_406_show_wf(zout), a_vw ); // ## 12
  stapl::do_once( msg( zout, "") );

  return res;
}


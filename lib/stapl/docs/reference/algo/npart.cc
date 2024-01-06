// algo/npart.cc

#include <stapl/runtime.hpp>
#include <stapl/containers/array/array.hpp>
#include <stapl/views/array_view.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/algorithms/sorting.hpp>

#include "algohelp.hpp"
using namespace std;

struct segment_wf {
  typedef void result_type;
  template<typename Ref1, typename Ref2>
  result_type operator() (Ref1 x, Ref2 y) {
    // process segment
  }
};

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;
typedef stapl::array<val_tp> ary_int_tp;
typedef stapl::array_view<ary_int_tp> ary_int_vw_tp;

val_tp data[] = {
  33,  44,  79,  64,  67,  80,   2,  84,  45,  94,
  89,   6,  11,  34,  40,  74,   0,  93,   5,  60,
  82,  41,  13,  69,   7,  26,  66,   9,  59,  63,
  76,  21,  30,  19,  25,  88,  42,  17,  77,  16,
  71,   1,  83,  35,  15,  97,   4,  75,  90,  52,
  56,  70,  38,  14,  53,  85,  78,  12,  10,  54,
  27,  95,  22,  18,  23,  39,   3,  98,  92,  57,
  68,  32,  99,  81,  37,  96,  91,  62,  50,  24,
  55,  28,  29,  61,  58,  72,  31,  43,  87,  65,
  49,  20,  46,  73,  36,  86,   8,  51,  48,  47,
};

int spl_vec[] = { 1, 2, 3, 5, 8, 11, 1, 2, 3, 4}; // sum is 40

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct containers and views
  vec_int_tp v_ct(40), w_ct(40), u_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct), u_vw(u_ct);

  // initialize containers
  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(w_vw, repeat_wf(base,rep));

  // construct containers
  vec_int_tp x_ct(40), y_ct(40);

  // copy random values to x_ct
  for( int i=0; i<40; i++ ) {
    x_ct[i] = data[i];
  }

  // copy splitter values to spl_ct;
  ary_int_tp spl_ct(10);
  for( int i=0; i<10; i++ ) {
    spl_ct[i] = data[i];
  }
  ary_int_vw_tp spl_vw(spl_ct);

  // construct view over container
  vec_int_vw_tp x_vw(x_ct), y_vw(y_ct);

  // initialize data values
  stapl::generate(x_vw, stapl::random_sequence());
  stapl::generate(y_vw, stapl::random_sequence());

  // apply algorithm
  auto p_vw = stapl::n_partition(u_vw, spl_vw );
  auto q_vw = stapl::n_partition(v_vw, spl_vw, stapl::less<val_tp>() );
  auto r_vw = stapl::n_partition(u_vw, spl_vw,
              stapl::less<val_tp>(), segment_wf() );

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_npart.txt");

  stapl::do_once( msg_val<size_t>( zout, "p_vw", p_vw.size() ) );
  stapl::do_once( msg_val<size_t>( zout, "q_vw", q_vw.size() ) );
  stapl::do_once( msg_val<size_t>( zout, "r_vw", r_vw.size() ) );

  return EXIT_SUCCESS;
}

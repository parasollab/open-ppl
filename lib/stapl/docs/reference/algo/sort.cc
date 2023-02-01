// algo/sort.cc

#include <stapl/runtime.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>
#include <stapl/algorithms/sorting.hpp>

#include "algohelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

val_tp data[] = {
  91,  65,   0,  46,  58,  82,  75,  98,  39,  27,
  74,  50,  87,  95,  80,  66,   2,  73,  97,  44,
  89,  86,  83,  15,  79,   6,  84,  70,  71,  16,
  29,  43,   5,  23,  53,   7,  62,  48,  61,  77,
  57,  18,  69,  13,  32,  38,  42,   9,  26,  45,
  76,  10,  24,  49,  96,  19,  30,  20,  40,  52,
  93,   3,  21,  17,  33,  88,  14,  36,  28,  34,
  35,  51,  78,  94,  68,  25,  41,  59,  99,  54,
  64,  55,  60,  31,  12,  90,   4,   1,  22,  92,
  72,  56,  63,  11,  81,  37,  67,  85,   8,  47,
};

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct containers
  vec_int_tp v_ct(40), w_ct(40);

  // copy random values to v_ct
  for( int i=0; i<40; i++ ) {
    w_ct[i] = data[i];
  }
  stapl::rmi_fence();

  // construct views over containers
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct);
  stapl::generate(v_vw, stapl::random_sequence());

  // apply algorithm
  stapl::sort(v_vw, stapl::greater<val_tp>() );
  stapl::sort(w_vw);

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_sort.txt");

  stapl::do_once( msg( zout, "v_vw") );
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg( zout, "w_vw") );
  stapl::serial_io( put_val_wf(zout), w_vw );

  return EXIT_SUCCESS;
}

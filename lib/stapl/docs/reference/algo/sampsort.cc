// algo/sampsort.cc

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
  37,  84,  92,  32,  25,  16,  78,  83,  40,   7,
  76,  85,  18,  52,  46,  23,  31,  94,  53,  50,
  59,  91,   8,   9,  38,  75,  34,  26,  39,  20,
  55,  79,  62,  97,  15,  90,  72,  98,  30,  35,
  42,  51,  87,  41,  21,  17,  89,  99,  93,  80,
  77,  64,  74,  57,   2,  82,  44,  43,  12,  13,
  28,  60,  56,   0,  81,  27,  61,  48,  70,  67,
   3,  10,  69,  45,  33,  49,  71,  36,  63,   4,
  22,  88,  19,  54,  11,  24,  58,   5,  68,  73,
  86,  66,  29,  95,  47,   1,  96,  14,  65,   6,
};

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct containers
  vec_int_tp v_ct(40), w_ct(40), u_ct(40);

  // copy random values to v_ct
  for( int i=0; i < 40; i++ ) {
    w_ct[i] = data[i];
  }
  stapl::rmi_fence();

  // construct views over containers
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct), u_vw(u_ct);

  stapl::generate(w_vw, stapl::random_sequence());

  // apply algorithm
  stapl::sample_sort(v_vw, stapl::greater<val_tp>() );
  stapl::sample_sort(w_vw);

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_sampsort.txt");

  stapl::do_once( msg( zout, "v_vw" ) );
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg( zout, "w_vw" ) );
  stapl::serial_io( put_val_wf(zout), w_vw );

  return EXIT_SUCCESS;
}

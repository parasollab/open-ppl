// algo/keepif.cc

#include <stapl/runtime.hpp>
#include <stapl/containers/vector/vector.hpp>
#include <stapl/views/vector_view.hpp>
#include <stapl/algorithms/algorithm.hpp>
#include <stapl/algorithms/functional.hpp>

#include "algohelp.hpp"
using namespace std;

typedef int val_tp;
typedef stapl::vector<val_tp> vec_int_tp;
typedef stapl::vector_view<vec_int_tp> vec_int_vw_tp;

template <typename T>
struct even_pred
{
  typedef T    argument_type;
  typedef bool result_type;
  result_type operator() (const T& x) const
  {
    return 0 == (x % 2);
  }
};

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct container and view over it
  vec_int_tp v_ct(30);
  vec_int_vw_tp v_vw(v_ct);

  // initialize container
  int base = 0;
  int step = 5;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  // apply algorithm
  stapl::keep_if( v_vw, even_pred<val_tp>() );

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_keepif.txt");
  stapl::do_once( msg( zout, "v_vw" ) );
  stapl::serial_io( put_val_wf(zout), v_vw );

  return EXIT_SUCCESS;
}

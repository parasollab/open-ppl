// algo/uniqcopy.cc

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
struct eq_pred
{
  typedef T    argument_type;
  typedef bool result_type;
  result_type operator() (const T& x, const T& y) const
  {
    return x == y;
  }
};

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct containers and views
  vec_int_tp v_ct(40), w_ct(40), u_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct), u_vw(u_ct);

  // initialize containers
  int base = 10;
  int step = 1;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(v_vw, step_wf(base,step));

  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(v_vw, repeat_wf(base,rep));

  // apply algorithms
  stapl::unique_copy(v_vw, u_vw, eq_pred<val_tp>());
  stapl::unique_copy(v_vw, w_vw);

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_uniqcopy.txt");

  stapl::do_once( msg( zout, "w_vw" ) );
  stapl::serial_io( put_val_wf(zout), w_vw );
  stapl::do_once( msg( zout, "u_vw" ) );
  stapl::serial_io( put_val_wf(zout), u_vw );

  return EXIT_SUCCESS;
}

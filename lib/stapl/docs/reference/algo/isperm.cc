// algo/isperm.cc

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
  stapl::iota(v_vw, 0);

  stapl::iota(w_vw, 1);
  w_vw[w_vw.size()-1] = 0;

  int base = 1;
  int step = 1;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(u_vw, step_wf(base,step));

  // apply algorithm
  bool test_p= false, test= false;
  test_p = stapl::is_permutation(v_vw, w_vw, stapl::equal_to<val_tp>() );
  test = stapl::is_permutation(w_vw, u_vw);

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_isperm.txt");
  stapl::serial_io( put_val_wf(zout), v_vw );
  stapl::do_once( msg_val<bool>( zout, "test_p ", test_p ) );
  stapl::do_once( msg_val<bool>( zout, "test ", test ) );

  return EXIT_SUCCESS;
}

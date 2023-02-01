// algo/issort.cc

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
struct less_pred
{
  typedef T    argument_type;
  typedef bool result_type;
  result_type operator() (const T& x, const T& y) const
  {
    return x < y;
  }
};

stapl::exit_code stapl_main(int argc, char **argv) {

  // construct containers and views
  vec_int_tp v_ct(40), w_ct(40), x_ct(40), y_ct(40);
  vec_int_vw_tp v_vw(v_ct), w_vw(w_ct), x_vw(x_ct), y_vw(y_ct);

  // initialize containers
  stapl::iota(v_vw, 0);

  int base = 0;
  int step = 10;
  typedef stapl::sequence<val_tp> step_wf;
  stapl::generate(w_vw, step_wf(base,step));

  int rep = 7;
  typedef stapl::block_sequence<int> repeat_wf;
  stapl::generate(x_vw, repeat_wf(base,rep));

  stapl::generate(y_vw, stapl::random_sequence());

  // apply algorithm
  bool w_test = stapl::is_sorted(w_vw);
  bool v_test = stapl::is_sorted(v_vw, stapl::less<val_tp>() );
  auto p_vw = stapl::is_sorted_until(y_vw);
  auto q_vw = stapl::is_sorted_until(x_vw, stapl::less<val_tp>() );

  // print results
  stapl::stream<ofstream> zout;
  zout.open("refman_issort.txt");
  stapl::do_once( msg_val<bool>( zout, "v_test", v_test ) );
  stapl::do_once( msg_val<bool>( zout, "w_test", w_test ) );

  return EXIT_SUCCESS;
}
